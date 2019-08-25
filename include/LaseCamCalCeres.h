//
// Created by heyijia on 18-12-11.
//

#ifndef PROJECT_LASECAMCALCERES_H
#define PROJECT_LASECAMCALCERES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Oberserve
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Oberserve()
    {
        tagPose_Qca = Eigen::Quaterniond(1,0,0,0);
        tagPose_tca = Eigen::Vector3d::Zero();
    }

    Eigen::Quaterniond tagPose_Qca;    // 该时刻 tag 的姿态
    Eigen::Vector3d tagPose_tca;
    std::vector<Eigen::Vector3d> points;   // 该时刻激光点云的数据
    std::vector<Eigen::Vector3d> points_on_line;   // 该时刻激光点云的数据
};

void LineFittingCeres(const std::vector<Eigen::Vector3d> Points, Eigen::Vector2d & Line);
void CamLaserCalibration(const std::vector<Oberserve> obs, Eigen::Matrix4d &Trc, bool use_linefitting_data = true, bool use_boundary_constraint = false);
void CalibrationTool_SavePlanePoints(const std::vector<Oberserve> obs, const Eigen::Matrix4d Tcl, const std::string path);
#endif //PROJECT_LASECAMCALCERES_H
