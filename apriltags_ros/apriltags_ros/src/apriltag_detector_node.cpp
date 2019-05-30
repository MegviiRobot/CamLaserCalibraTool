#include <apriltags_ros/apriltag_detector.h>
#include <ros/ros.h>

int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_detector");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  apriltags_ros::AprilTagDetector detector(nh, pnh);
  ros::spin();
}
