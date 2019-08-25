#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "../camera_models/include/EquidistantCamera.h"
#include "../camera_models/include/PinholeCamera.h"
#include "config.h"
#include "utilities.h"
#include "selectScanPoints.h"
#include "calcCamPose.h"
#include "LaseCamCalCeres.h"

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  std::cout << name <<std::endl;
  T ans;
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

class kalibraTagDetector{
public:
    kalibraTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~kalibraTagDetector();
//    bool initParam(std::string filename);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    bool detect(const sensor_msgs::ImageConstPtr& msg, CamPose& T);
private:
    CameraPtr cameraptr_;
    cv::Mat undist_map1_,undist_map2_;
    ros::NodeHandle it_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::Publisher detections_pub_;
    ros::Publisher pose_pub_;
};

kalibraTagDetector::kalibraTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){

  std::string config_file;
  config_file = readParam<std::string>(pnh, "config_file");

  cv::FileStorage fsSettings(config_file,     cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  std::string sModelType;
  fsSettings["model_type"] >> sModelType;

  cameraptr_ = nullptr;
  if (sModelType.compare("KANNALA_BRANDT") == 0)
  {
    EquidistantCamera::Parameters paras;
    paras.readFromYamlFile(config_file);
    cameraptr_ = CameraPtr (new EquidistantCamera(paras));
//    cameraptr_->initUndistortRectifyMap(undist_map1_,undist_map2_,paras.mu(),paras.mv(),cv::Size(paras.imageWidth(), paras.imageHeight()),paras.u0(),paras.v0());
    ROS_INFO("LOAD KANNALA BRANDT CAMERA!");
  }
  // todo:: PIN HOLE MODEL
  else
  {
    PinholeCamera::Parameters paras;
    paras.readFromYamlFile(config_file);
    cameraptr_ = CameraPtr (new PinholeCamera(paras));
//    cameraptr_->initUndistortRectifyMap(undist_map1_,undist_map2_,paras.fx(),paras.fy(),cv::Size(paras.imageWidth(), paras.imageHeight()),paras.cx(),paras.cy());
  }

  std::string img_topic_name = "image_rect";
  fsSettings["img_topic_name"] >> img_topic_name;

  fsSettings.release();

  image_sub_ = it_.subscribe(img_topic_name,10, &kalibraTagDetector::imageCb, this);
}

kalibraTagDetector::~kalibraTagDetector(){
  image_sub_.shutdown();
}


//bool kalibraTagDetector::initParam(std::string filename)
//{
//
//}

void kalibraTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
//  cv::imshow("im", cv_ptr->image);
//  cv::waitKey(1);

  Eigen::Matrix4d Twc;
  if (cameraptr_ != nullptr)
  {
    // 直接传原始图像，也可以检测二维码。里面会对坐标去畸变再计算 pose.
    cv::Mat rectified = cv_ptr->image.clone();
//    cv::remap(cv_ptr->image, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    calcCamPose(cv_ptr->header.stamp.toSec(), rectified, cameraptr_, Twc);
  }
  else{
    ROS_ERROR("please select camera model and write in yaml file");
  }
}

bool kalibraTagDetector::detect(const sensor_msgs::ImageConstPtr& msg, CamPose& T){
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
//  cv::imshow("im", cv_ptr->image);
//  cv::waitKey(1);

  Eigen::Matrix4d Twc;
  if (cameraptr_ != nullptr)
  {
    // 直接传原始图像，也可以检测二维码。里面会对坐标去畸变再计算 pose.
    cv::Mat rectified = cv_ptr->image.clone();
//    cv::remap(cv_ptr->image, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    calcCamPose(cv_ptr->header.stamp.toSec(), rectified, cameraptr_, Twc);

    T.timestamp = cv_ptr->header.stamp.toSec();
    T.twc = Twc.block(0,3,3,1);
    Eigen::Matrix3d R = Twc.block(0,0,3,3);
    Eigen::Quaterniond q(R);
    T.qwc = q;
  }
  else{
    ROS_ERROR("please select camera model and write in yaml file");
    return false;
  }
}
int main(int argc, char **argv){
  ros::init(argc, argv, "LaserCamCal");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string config_file;
  config_file = readParam<std::string>(pnh, "config_file");
  readParameters(config_file);

  rosbag::Bag bag_input;
  bag_input.open(bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics;
  topics.push_back(scan_topic_name);
  topics.push_back(img_topic_name);
  rosbag::View views(bag_input, rosbag::TopicQuery(topics));

  // 初始视觉 pose
  kalibraTagDetector detector(nh, pnh);
  std::vector < CamPose > tagpose;
  int i = 0;
  for (rosbag::MessageInstance const m: views)
  {

//    if(i%4 != 0) {
//      i++;
//      continue;
//    }

      if (m.getTopic() == img_topic_name)
      {
        sensor_msgs::ImageConstPtr img = m.instantiate<sensor_msgs::Image>();
//        detector.imageCb(img);
        CamPose Twc;
        if(detector.detect(img, Twc))
        {
          tagpose.push_back(Twc);
        }

      }
  }

  // 准备标定数据
  std::vector<Oberserve> obs;

  // 处理激光数据
  for(rosbag::MessageInstance const m: views){
    if (m.getTopic() == scan_topic_name)
    {
      sensor_msgs::LaserScan::Ptr scan = m.instantiate<sensor_msgs::LaserScan>();
      std::vector<Eigen::Vector3d> Points;
      TranScanToPoints(*scan,Points);

      double timestamp = scan->header.stamp.toSec();
      std::vector<Eigen::Vector3d> points;

      points = AutoGetLinePts(Points);

      // 检测到了直线
      if(points.size() > 30)
      {
        // 在 camera 里找时间戳最近的一个 pose
        double min_dt = 10000;
        CamPose colsetTagPose;
        for (int i = 0; i < tagpose.size(); ++i) {
          CamPose tmp = tagpose.at(i);
          double t = fabs(tmp.timestamp - timestamp);
          if(t < min_dt)
          {
            min_dt = t;
            colsetTagPose = tmp;
          }
        }

        std::cout << "scan and tag time: "<<std::fixed<<std::setprecision(18)
                  <<timestamp<<" "<<colsetTagPose.timestamp<<std::endl;
        if(min_dt < 0.02)  // 20ms
        {
          /////////////////////////////////////////////////
          // 用这一堆数据，拟合一条直线，之前的约束假设他们是在平面上，约束还不够强
          double x_min(1000.0), x_max(-1000.), y_min(1000.), y_max(-1000.);
          for (int i = 0; i < points.size(); ++i) {
            Eigen::Vector3d pt = points[i];
            if(pt.x() < x_min)
              x_min = pt.x();
            if(pt.x() > x_max)
              x_max = pt.x();

            if(pt.y() < y_min)
              y_min = pt.y();
            if(pt.y() > y_max)
              y_max = pt.y();
          }
          Eigen::Vector2d line;
          LineFittingCeres(points,line);
          std::vector<Eigen::Vector3d> points_on_line;

          if(x_max - x_min > y_max - y_min)
          {
            y_min = - (x_min * line(0) + 1) / line(1);
            y_max = - (x_max * line(0) + 1) / line(1);

          } else // 垂直于 x 轴
          {
            x_min = - (y_min * line(1) + 1) / line(0);
            x_max = - (y_max * line(1) + 1) / line(0);
          }

          points_on_line.push_back(Eigen::Vector3d(x_min,y_min,0));
          points_on_line.push_back(Eigen::Vector3d(x_max,y_max,0));

          Oberserve ob;
          ob.tagPose_Qca = colsetTagPose.qwc;
          ob.tagPose_tca = colsetTagPose.twc;
          ob.points = points;
          ob.points_on_line = points_on_line;
          obs.push_back(ob);
        }

      }


    }
  }


  if(obs.size() < 5)
  {
    std::cout << "Valid Calibra Data Less"<<std::endl;
    bag_input.close();
    return 0;
  }
  std::cout <<"obs size: "<< obs.size() <<std::endl;

  Eigen::Matrix4d Tcl = Eigen::Matrix4d::Identity();
  CamLaserCalibration(obs,Tcl,true);

  std::cout << "\n----- Transform from Camera to Laser Tlc is: -----\n"<<std::endl;
  std::cout<<Tcl.inverse()<<std::endl;

  // save to yaml file
  cv::Mat cvTlc;
  Eigen::Matrix4d Tlc = Tcl.inverse();
  cv::eigen2cv(Tlc,cvTlc);
  std::string fn = savePath + "result.yaml";
  cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  fs << "extrinsicTlc"<<cvTlc;
  fs.release();
  std::cout << "\n Result file : "<<fn<<std::endl;
  std::cout << "\n-------------- Calibration Code End --------------\n"<<std::endl;


//  EquidistantCamera::Parameters paras;
//  paras.readFromYamlFile("/home/heyijia/ros_ws/apriltag_ws/src/kalibra_tag_ws/config/calibra_config.yaml");
//  std::cout <<paras.cameraName()<<" "<<paras.imageWidth() <<" "<<paras.k2() <<std::endl;
//  CameraPtr pinCameraPtr(new EquidistantCamera(paras));

//  kalibraTagDetector detector(nh, pnh);


  ros::spin();
}
