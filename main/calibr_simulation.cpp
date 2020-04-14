#include <ros/ros.h>

#include <random>

#include "utilities.h"
#include "calcCamPose.h"
#include "LaseCamCalCeres.h"


void GenerateSimData( std::vector<Oberserve>& obs)
{

  // we set laser frame as the world frame
  // generate Tlc: Transformation matrix from camera frame to lasr frame;
  Eigen::Matrix3d Rlc;
  Rlc << 0 , 0 , 1,
        -1 , 0 , 0,
        0 , -1 , 0;
  Eigen::Quaterniond qlc(Rlc);              
  Eigen::Vector3d tlc(0.1, 0.2, 0.3);

  std::cout << "============= Ground Truth of Rotation and translation: Rlc, tlc =============== " << std::endl;
  std::cout << Rlc << std::endl;
  std::cout << tlc << std::endl;
  std::cout << "============= End =============== " << std::endl;

  std::random_device rd;
  std::default_random_engine generator(rd());
  // random CamPose distribution
  std::uniform_real_distribution<double> rpy_rand(-M_PI/6. , M_PI/6.);
  std::uniform_real_distribution<double> xy_rand(-3, 3.0);
  std::uniform_real_distribution<double> z_rand(1., 5.);

  for (size_t i = 0; i < 50; i++)
  {
    // generate apriltage pose in camera frame, then transfrom it to world frame
    Eigen::Matrix3d Rca;
    
////////////////////////////// Try it!!! You will find the treasure /////////////////////////////////////////////////////////////////    
    // You can try rotating the calibration plate around the camera's different axes to see the observability of the system.
    // rotate all axis
    Rca = Eigen::AngleAxisd(rpy_rand(generator),Eigen::Vector3d::UnitZ())
        *Eigen::AngleAxisd(rpy_rand(generator),Eigen::Vector3d::UnitY())
        *Eigen::AngleAxisd(rpy_rand(generator),Eigen::Vector3d::UnitX());

    // Try me!!! remove yaw, you will find that the system observability is not affected by Z-axis
    // Rca = Eigen::AngleAxisd(rpy_rand(generator),Eigen::Vector3d::UnitY())
    //     *Eigen::AngleAxisd(rpy_rand(generator),Eigen::Vector3d::UnitX());
      
    // Try me!!! ONLY pitch, Wow!!!! you will find we can not estimate the tlc.z() 
    // Rca = Eigen::AngleAxisd(rpy_rand(generator),Eigen::Vector3d::UnitY());

    // Try me!!! ONLY roll
    // Rca = Eigen::AngleAxisd(rpy_rand(generator),Eigen::Vector3d::UnitX());

///////////////////////////////////////////////////////////////////////////////////////////////

    Eigen::Vector3d tca(xy_rand(generator),xy_rand(generator), z_rand(generator));
    // Eigen::Vector3d tca(0,0, z_rand(generator));
    Eigen::Quaterniond qca(Rca);

    Eigen::Quaterniond qla(Rlc * Rca);
    Eigen::Vector3d tla = Rlc * tca + tlc;

    // get apriltag plane equation , then transform to laser frame.
    Eigen::Vector4d plane_in_aprilframe(0,0,1,0); 

    Eigen::Matrix4d Tla = Eigen::Matrix4d::Identity();
    Tla.block(0, 0, 3, 3) = qla.toRotationMatrix();
    Tla.block(0, 3, 3, 1) = tla;
    Eigen::Vector4d plane_in_laserframe = (Tla.inverse()).transpose() * plane_in_aprilframe;
    Eigen::Vector3d n = plane_in_laserframe.head(3);
    double d = plane_in_laserframe[3];


    // Obtaining laser observations through the intersection of rays and planes
    std::vector< Eigen::Vector3d > points;

    for (size_t j = 0; j < 180; j++)
    {
      double theta = -M_PI_2 + j * M_PI/180;
      Eigen::Vector3d ray(cos(theta), sin(theta), 0);
      double depth = -d/(ray.dot(n));
      
      if( std::isnan(depth) || depth < 0)
        continue;
      
      Eigen::Vector3d p = depth * ray;
      
      // valid laser points
      if( std::fabs(p.x()) < 5 && std::fabs(p.y()) < 5)
      {
          points.push_back(p);
      }
      // std::cout << p.transpose() << std::endl;
    }

    Oberserve ob;
    ob.tagPose_Qca = qca;
    ob.tagPose_tca = tca;
    ob.points = points;
    ob.points_on_line = points;
    obs.push_back(ob);    

  }
  

}

int main(int argc, char **argv){
  ros::init(argc, argv, "LaserCamCal_Simulation");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // Prepare Simulation data
  std::vector<Oberserve> obs;
  GenerateSimData(obs);

  if(obs.size() < 5)
  {
    std::cout << "Valid Calibra Data Less"<<std::endl;
    return 0;
  }
  std::cout <<"obs size: "<< obs.size() <<std::endl;

  Eigen::Matrix4d Tlc_initial = Eigen::Matrix4d::Identity();
  // CamLaserCalClosedSolution(obs,Tlc_initial);

  Eigen::Matrix4d Tcl = Tlc_initial.inverse();
  CamLaserCalibration(obs,Tcl, false);   // refine results by non-linear optimiztion
  //CamLaserCalibration(obs,Tcl, true);

  std::cout << "\n----- Transform from Camera to Laser Tlc is: -----\n"<<std::endl;
  Eigen::Matrix4d Tlc = Tcl.inverse();
  std::cout<< Tlc <<std::endl;

  std::cout << "\n----- Transform from Camera to Laser, euler angles and translations are: -----\n"<<std::endl;
  Eigen::Matrix3d Rlc(Tlc.block(0,0,3,3));
  Eigen::Vector3d tlc(Tlc.block(0,3,3,1));
  EulerAngles rpy =  ToEulerAngles(Eigen::Quaterniond(Rlc));
  std::cout << "   roll(rad): "<<rpy.roll <<" pitch(rad): "<<rpy.pitch << " yaw(rad): "<<rpy.yaw<<"\n"
            << "or roll(deg): "<<rpy.roll * 180./M_PI <<" pitch(deg): "<<rpy.pitch* 180./M_PI  << " yaw(deg): "<<rpy.yaw * 180./M_PI <<"\n"
            << "       tx(m): "<<tlc.x() << "  ty(m): "<<tlc.y() << "   tz(m): "<<tlc.z()<<std::endl;

  // save to yaml file
  // cv::Mat cvTlc;
  // cv::eigen2cv(Tlc,cvTlc);
  // std::string fn = savePath + "result.yaml";
  // cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  // fs << "extrinsicTlc"<<cvTlc;
  // cv::Mat cvrpy;
  // cv::eigen2cv(Eigen::Vector3d(rpy.roll,rpy.pitch,rpy.yaw),cvrpy);
  // cv::Mat cvtlc;
  // cv::eigen2cv(tlc,cvtlc);
  // fs << "RollPitchYaw"<<cvrpy;
  // fs << "txtytz"<<cvtlc;
  // fs.release();


  // std::cout << "\n Result file : "<<fn<<std::endl;
  // std::cout << "\n-------------- Calibration Code End --------------\n"<<std::endl;


  ros::spin();
}
