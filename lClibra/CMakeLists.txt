cmake_minimum_required(VERSION 2.8.3)
project(lClibra)
SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo , Debug


#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 ")


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(PACKAGE_DEPENDENCIES
  roscpp
  rosbag
  sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
#  DEPENDS Eigen OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

file(GLOB_RECURSE PROJECT_SRCS src/*.cpp)

add_executable(${PROJECT_NAME}
  ${PROJECT_SRCS}
)
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)
