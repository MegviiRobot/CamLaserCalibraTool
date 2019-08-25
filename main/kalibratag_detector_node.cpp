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
    CamPoseEst camposecal_;
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

    int tagtype;
    double tagsize = 0.055;  // default size
    double tagspace = 0.3;  // default
    int black_border;
    tagtype = static_cast<int>(fsSettings["tag_type"]);
    tagsize = static_cast<double>(fsSettings["tag_size"]);
    tagspace = static_cast<double>(fsSettings["tag_spacing"]);
    black_border = static_cast<double>(fsSettings["black_border"]);
    std::cout <<"tag_size: "<< tagsize <<" type: "<<tagtype<<std::endl;

    if( 1 == tagtype)
    {
        CalibrBoardInfo info(KALIBR_TAG_PATTERN,tagsize,tagspace,black_border);
        camposecal_ = CamPoseEst(info);
    }else if(2 == tagtype)
    {
        CalibrBoardInfo info(APRIL_TAG_ONE,tagsize,black_border);
        camposecal_ = CamPoseEst(info);
    }else if (3 == tagtype)
    {
        CalibrBoardInfo info(CHESS,tagsize);
        camposecal_ = CamPoseEst(info);
    }

    std::string sModelType;
    fsSettings["model_type"] >> sModelType;

    cameraptr_ = nullptr;
    if (sModelType.compare("KANNALA_BRANDT") == 0)
    {
        EquidistantCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new EquidistantCamera(paras));
        cameraptr_->initUndistortRectifyMap(undist_map1_,undist_map2_,paras.mu(),paras.mv(),cv::Size(paras.imageWidth(), paras.imageHeight()),paras.u0(),paras.v0());
        ROS_INFO("LOAD KANNALA BRANDT CAMERA!");
    }
    else if(sModelType.compare("PINHOLE") == 0)
    {
        PinholeCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new PinholeCamera(paras));
        cameraptr_->initUndistortRectifyMap(undist_map1_,undist_map2_,paras.fx(),paras.fy(),cv::Size(paras.imageWidth(), paras.imageHeight()),paras.cx(),paras.cy());
        ROS_INFO("LOAD PINHOLE CAMERA!"); 
   }

    std::string img_topic_name = "image";
    fsSettings["img_topic_name"] >> img_topic_name;

    fsSettings.release();

    image_sub_ = it_.subscribe(img_topic_name,10, &kalibraTagDetector::imageCb, this);
}

kalibraTagDetector::~kalibraTagDetector(){
  image_sub_.shutdown();
}

void kalibraTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Eigen::Matrix4d Twc;
  if (cameraptr_ != nullptr)
  {
    // 直接传原始图像，也可以检测二维码。里面会对坐标去畸变再计算 pose.
    cv::Mat rectified = cv_ptr->image.clone();
//    cv::remap(cv_ptr->image, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
//      cv::imshow("rect",rectified);
//      cv::waitKey(1);
    camposecal_.calcCamPose(cv_ptr->header.stamp.toSec(), rectified, cameraptr_, Twc);
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

  Eigen::Matrix4d Twc;
  if (cameraptr_ != nullptr)
  {
    // 直接传原始图像，也可以检测二维码。里面会对坐标去畸变再计算 pose.
    cv::Mat rectified = cv_ptr->image.clone();
    //cv::remap(cv_ptr->image, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    bool success_flag = camposecal_.calcCamPose(cv_ptr->header.stamp.toSec(), rectified, cameraptr_, Twc);

    T.timestamp = cv_ptr->header.stamp.toSec();
    T.twc = Twc.block(0,3,3,1);
    Eigen::Matrix3d R = Twc.block(0,0,3,3);
    Eigen::Quaterniond q(R);
    T.qwc = q;
    if(success_flag)
    {
//        std::cout <<"Tcw:\n"<< Twc.inverse() << std::endl;
    }

    return success_flag;
  }
  else{
    ROS_ERROR("please select camera model and write in yaml file");
    return false;
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "CamPoseEstimation");
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
    std::string file = savePath + "apriltag_pose.txt";
    std::ofstream fC(file.c_str());
    fC.close();
    std::ofstream foutC(file.c_str(), std::ios::app);
    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(9);
    for (rosbag::MessageInstance const m: views)
    {
      if (m.getTopic() == img_topic_name)
      {
        sensor_msgs::ImageConstPtr img = m.instantiate<sensor_msgs::Image>();
        CamPose Twc;
        if(detector.detect(img, Twc))
        {
            tagpose.push_back(Twc);

            EulerAngles rpy =  ToEulerAngles(Twc.qwc);

            foutC << Twc.timestamp<< " ";
            foutC.precision(10);
            foutC << Twc.twc(0) << " "
                  << Twc.twc(1) << " "
                  << Twc.twc(2) << " "
                  << Twc.qwc.x() << " "
                  << Twc.qwc.y() << " "
                  << Twc.qwc.z() << " "
                  << Twc.qwc.w() << " "
                  << rpy.roll << " "
                  << rpy.pitch<< " "
                  << rpy.yaw
                  << std::endl;
        }
      }
    }
    foutC.close();
  ros::spin();
}
