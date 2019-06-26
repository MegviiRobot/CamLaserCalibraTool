#include <apriltags_ros/apriltag_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>

#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/Tag25h7.h>
#include <AprilTags/Tag25h9.h>
#include <AprilTags/Tag36h9.h>
#include <AprilTags/Tag36h11.h>
#include <XmlRpcException.h>

#include <iostream>
#include <fstream>

namespace apriltags_ros{

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

AprilTagDetector::AprilTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){
  XmlRpc::XmlRpcValue april_tag_descriptions;
  if(!pnh.getParam("tag_descriptions", april_tag_descriptions)){
    ROS_WARN("No april tags specified");
  }
  else{
    try{
      descriptions_ = parse_tag_descriptions(april_tag_descriptions);
    } catch(XmlRpc::XmlRpcException e){
      ROS_ERROR_STREAM("Error loading tag descriptions: "<<e.getMessage());
    }
  }

  if(!pnh.getParam("sensor_frame_id", sensor_frame_id_)){
    sensor_frame_id_ = "";
  }

  std::string tag_family;
  pnh.param<std::string>("tag_family", tag_family, "36h11");

  pnh.param<bool>("projected_optics", projected_optics_, false);

  const AprilTags::TagCodes* tag_codes;
  if(tag_family == "16h5"){
    tag_codes = &AprilTags::tagCodes16h5;
  }
  else if(tag_family == "25h7"){
    tag_codes = &AprilTags::tagCodes25h7;
  }
  else if(tag_family == "25h9"){
    tag_codes = &AprilTags::tagCodes25h9;
  }
  else if(tag_family == "36h9"){
    tag_codes = &AprilTags::tagCodes36h9;
  }
  else if(tag_family == "36h11"){
    tag_codes = &AprilTags::tagCodes36h11;
  }
  else{
    ROS_WARN("Invalid tag family specified; defaulting to 36h11");
    tag_codes = &AprilTags::tagCodes36h11;
  }


    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    std::string img_topic_name = "image_rect";
    fsSettings["img_topic_name"] >> img_topic_name;
    fsSettings["savePath"] >> pose_save_path_;

    int width_ = fsSettings["image_width"];
    int height_ = fsSettings["image_height"];
    cv::FileNode n1 = fsSettings["distortion_parameters"];
    double m_k1 = static_cast<double>(n1["k1"]);
    double m_k2 = static_cast<double>(n1["k2"]);
    double m_p1 = static_cast<double>(n1["p1"]);
    double m_p2 = static_cast<double>(n1["p2"]);
    n1 = fsSettings["projection_parameters"];
    double m_fx = static_cast<double>(n1["fx"]);
    double m_fy = static_cast<double>(n1["fy"]);
    double m_cx = static_cast<double>(n1["cx"]);
    double m_cy = static_cast<double>(n1["cy"]);
    fsSettings.release();

    cvK_ = (cv::Mat_<float>(3, 3) << m_fx, 0.0, m_cx, 0.0, m_fy, m_cy, 0.0, 0.0, 1.0);
    cvD_ = (cv::Mat_<float>(1, 5) << m_k1, m_k2, m_p1, m_p2, 0.);
    cv::initUndistortRectifyMap(cvK_, cvD_, cv::Mat_<double>::eye(3,3), cvK_,
                                cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);

  std::cout << "Apriltag initial complete\n";

  tag_detector_= boost::shared_ptr<AprilTags::TagDetector>(new AprilTags::TagDetector(*tag_codes));
  image_sub_ = it_.subscribe(img_topic_name,10, &AprilTagDetector::imageCb, this);
  detections_pub_ = nh.advertise<AprilTagDetectionArray>("tag_detections", 1);
  pose_pub_ = nh.advertise<nav_msgs::Path>("tag_detections_pose", 1);

  std::string file = pose_save_path_ + "apriltag_pose.txt";
  std::ofstream foutC(file.c_str());
}
AprilTagDetector::~AprilTagDetector(){
  image_sub_.shutdown();
}

void AprilTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat gray, rectified;
  cv::remap(cv_ptr->image, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
  cv::cvtColor(rectified, gray, CV_BGR2GRAY);
  std::vector<AprilTags::TagDetection>	detections = tag_detector_->extractTags(gray);
  ROS_DEBUG("%d tag detected", (int)detections.size());

  double fx;
  double fy;
  double px;
  double py;
  fx = cvK_.at<float>(0,0);
  fy = cvK_.at<float>(1,1);
  px = cvK_.at<float>(0,2);
  py = cvK_.at<float>(1,2);
//  if (projected_optics_) {
//    // use projected focal length and principal point
//    // these are the correct values
//    fx = cam_info->P[0];
//    fy = cam_info->P[5];
//    px = cam_info->P[2];
//    py = cam_info->P[6];
//  } else {
//    // use camera intrinsic focal length and principal point
//    // for backwards compatability
//    fx = cam_info->K[0];
//    fy = cam_info->K[4];
//    px = cam_info->K[2];
//    py = cam_info->K[5];
//  }

  if(!sensor_frame_id_.empty())
    cv_ptr->header.frame_id = sensor_frame_id_;

  AprilTagDetectionArray tag_detection_array;
  nav_msgs::Path tag_pose_array;
  tag_pose_array.header = cv_ptr->header;

  BOOST_FOREACH(AprilTags::TagDetection detection, detections){
    std::map<int, AprilTagDescription>::const_iterator description_itr = descriptions_.find(detection.id);
    if(description_itr == descriptions_.end()){
      ROS_WARN_THROTTLE(10.0, "Found tag: %d, but no description was found for it", detection.id);
      continue;
    }
    AprilTagDescription description = description_itr->second;
    double tag_size = description.size();

    detection.draw(rectified);
    Eigen::Matrix4d transform = detection.getRelativeTransform(tag_size, fx, fy, px, py);
    Eigen::Matrix3d rot = transform.block(0, 0, 3, 3);
    Eigen::Quaternion<double> rot_quaternion = Eigen::Quaternion<double>(rot);

    geometry_msgs::PoseStamped tag_pose;
    tag_pose.pose.position.x = transform(0, 3);
    tag_pose.pose.position.y = transform(1, 3);
    tag_pose.pose.position.z = transform(2, 3);
    tag_pose.pose.orientation.x = rot_quaternion.x();
    tag_pose.pose.orientation.y = rot_quaternion.y();
    tag_pose.pose.orientation.z = rot_quaternion.z();
    tag_pose.pose.orientation.w = rot_quaternion.w();
    tag_pose.header = cv_ptr->header;
    tag_pose.header.seq = detection.id;

    std::string file = pose_save_path_ + "apriltag_pose.txt";
    std::ofstream foutC(file.c_str(), std::ios::app);
    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(9);
    foutC << cv_ptr->header.stamp.toSec()<< " ";
    foutC.precision(5);
    foutC << transform(0, 3) << " "
        << transform(1, 3) << " "
        << transform(2, 3) << " "
        << rot_quaternion.x() << " "
        << rot_quaternion.y() << " "
        << rot_quaternion.z() << " "
        << rot_quaternion.w() << std::endl;
    foutC.close();

    AprilTagDetection tag_detection;
    tag_detection.pose = tag_pose;
    tag_detection.id = detection.id;
    tag_detection.size = tag_size;
    tag_detection_array.detections.push_back(tag_detection);
    tag_pose_array.poses.push_back(tag_pose);

    tf::Stamped<tf::Transform> tag_transform;
    tf::poseStampedMsgToTF(tag_pose, tag_transform);
    tf_pub_.sendTransform(tf::StampedTransform(tag_transform, tag_transform.stamp_, tag_transform.frame_id_, description.frame_name()));
  }
  detections_pub_.publish(tag_detection_array);
  pose_pub_.publish(tag_pose_array);

  cv::imshow("tag_detection", rectified);
  cv::waitKey(1);
}


std::map<int, AprilTagDescription> AprilTagDetector::parse_tag_descriptions(XmlRpc::XmlRpcValue& tag_descriptions){
  std::map<int, AprilTagDescription> descriptions;
  ROS_ASSERT(tag_descriptions.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int32_t i = 0; i < tag_descriptions.size(); ++i) {
    XmlRpc::XmlRpcValue& tag_description = tag_descriptions[i];
    ROS_ASSERT(tag_description.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(tag_description["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(tag_description["size"].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    int id = (int)tag_description["id"];
    double size = (double)tag_description["size"];

    std::string frame_name;
    if(tag_description.hasMember("frame_id")){
      ROS_ASSERT(tag_description["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);
      frame_name = (std::string)tag_description["frame_id"];
    }
    else{
      std::stringstream frame_name_stream;
      frame_name_stream << "tag_" << id;
      frame_name = frame_name_stream.str();
    }
    AprilTagDescription description(id, size, frame_name);
    ROS_INFO_STREAM("Loaded tag config: "<<id<<", size: "<<size<<", frame_name: "<<frame_name);
    descriptions.insert(std::make_pair(id, description));
  }
  return descriptions;
}


}
