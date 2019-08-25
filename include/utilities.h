//
// Created by heyijia on 18-12-11.
//

#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <sensor_msgs/LaserScan.h>
struct CamPose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    double start_time;   // used to record the average pose start and end time
    double end_time;

    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

};

struct EulerAngles
{
    double yaw, pitch, roll;
};


Eigen::Vector4d pi_from_ppp(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3);
struct StraightLine
{
    Eigen::Vector2d A;
    Eigen::Vector2d B;
};
bool lineCrossPt(const StraightLine& l1, const StraightLine& l2, Eigen::Vector2d& pt);

EulerAngles ToEulerAngles(Eigen::Quaterniond q);
Eigen::Quaterniond yprToQuaternion(double yaw, double pitch, double roll);


void LoadCamPoseFromTxt(std::string filename, std::vector < CamPose >& allcampose );
void SaveCamPosetoTxt(std::string filename, const std::vector < CamPose > allcampose );


std::vector < CamPose > GetInversePose(std::vector < CamPose > pose);
Eigen::Quaterniond getAvergeQwc(Eigen::Matrix4d A);
std::vector< std::vector < CamPose > >  GetStaticPose(const std::vector < CamPose > Poses, std::vector< CamPose > &avergeStaticPoses);

void TranScanToPoints(const sensor_msgs::LaserScan& scan_in,std::vector<Eigen::Vector3d>& Points);

#endif //PROJECT_UTILITIES_H
