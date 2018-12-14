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

struct CamPose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double timestamp;
    double start_time;   // used to record the average pose start and end time
    double end_time;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;
};

void LoadCamPoseFromTxt(std::string filename, std::vector < CamPose >& allcampose );
void SaveCamPosetoTxt(std::string filename, const std::vector < CamPose > allcampose );


std::vector < CamPose > GetInversePose(std::vector < CamPose > pose);
Eigen::Quaterniond getAvergeQwc(Eigen::Matrix4d A);
std::vector< std::vector < CamPose > >  GetStaticPose(const std::vector < CamPose > Poses, std::vector< CamPose > &avergeStaticPoses);

#endif //PROJECT_UTILITIES_H
