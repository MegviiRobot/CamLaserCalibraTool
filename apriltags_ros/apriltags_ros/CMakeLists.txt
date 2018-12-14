cmake_minimum_required(VERSION 2.8.3)
project(apriltags_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  apriltags
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  message_generation
  nodelet
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)

add_message_files(
  FILES
  AprilTagDetection.msg
  AprilTagDetectionArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apriltag_detector
  CATKIN_DEPENDS apriltags image_transport roscpp sensor_msgs geometry_msgs tf cv_bridge message_runtime nodelet std_msgs
  DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(apriltag_detector
  src/apriltag_detector.cpp
)
target_link_libraries(apriltag_detector
  ${catkin_LIBRARIES}
)
add_dependencies(apriltag_detector apriltags_ros_generate_messages_cpp)


add_library(apriltag_detector_nodelet src/apriltag_detector_nodelet.cpp)
target_link_libraries(apriltag_detector_nodelet
  ${catkin_LIBRARIES}
  apriltag_detector
)

add_executable(apriltag_detector_node src/apriltag_detector_node.cpp)
target_link_libraries(apriltag_detector_node
  ${catkin_LIBRARIES}
  apriltag_detector
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS apriltag_detector apriltag_detector_nodelet apriltag_detector_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/apriltags_ros
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
