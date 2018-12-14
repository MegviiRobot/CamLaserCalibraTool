//
// Created by heyijia on 18-12-11.
//

#include <iostream>
#include <fstream>
#include <iomanip>
#include <cstdio>
#include <cstdlib>

#include "ceres/ceres.h"
#include "LaseCamCalCeres.h"


class CamScanPointsErr
{
private:
    Eigen::Vector4d planar_;
    Eigen::Vector3d point_;

public:
    CamScanPointsErr(Eigen::Vector4d planar,    // planar
                     Eigen::Vector3d point)   // point
            : planar_(planar),point_(point)
    {}

    template<typename T>
    bool operator() (const T* const q4x1 , const T* const t3x1, T* residuals) const
    {
        Eigen::Quaternion<T> qcl( q4x1[0], q4x1[1],q4x1[2],q4x1[3]);
        Eigen::Matrix<T,3,1> tcl;
        tcl<<t3x1[0],t3x1[1], t3x1[2];

        Eigen::Matrix<T,4,1> planar = planar_.cast<T>();
        Eigen::Matrix<T,3,1> pt_l = point_.cast<T>();

        Eigen::Matrix<T,3,1> pt_c = qcl.toRotationMatrix() * pt_l + tcl;

        T ndotp = planar[0] * pt_c[0] + planar[1] * pt_c[1] + planar[2] * pt_c[2] + planar[3];

        // norm_n always equal to 1
//        T norm_n = T(planar[0] * planar[0]) + T(planar[1] * planar[1]) + T(planar[2] * planar[2]);

        residuals[0] =  ndotp;// std::sqrt(norm_n);

        return true;
    }

};


void CalibrationTool_SavePlanePoints(const std::vector<Oberserve> obs, const Eigen::Matrix4d Tcl, const std::string path)
{

    std::ofstream fs_planar(path + "planar.txt");
    std::ofstream fs_points(path + "RoiPoints.txt");
    std::ofstream fs_lines(path + "RoiPtOnLines.txt");
    fs_planar << std::setprecision(3);
    fs_points << std::setprecision(3);
    fs_lines << std::setprecision(3);

    for(size_t i = 0; i< obs.size(); ++i) {
        Oberserve obi = obs[i];

        /// save planar
        // transform planar in tag frame to cam frame
        //https://stackoverflow.com/questions/7685495/transforming-a-3d-plane-using-a-4x4-matrix
        Eigen::Vector4d planar_tag(0, 0, 1, 0);  // tag 坐标系下的平面方程
        Eigen::Matrix4d Tctag = Eigen::Matrix4d::Identity();
        Tctag.block(0, 0, 3, 3) = obi.tagPose_Qca.toRotationMatrix();
        Tctag.block(0, 3, 3, 1) = obi.tagPose_tca;
        Eigen::Vector4d planar_cam = (Tctag.inverse()).transpose() * planar_tag;
        fs_planar<< i <<" "<< planar_cam[0] <<" "<< planar_cam[1] <<" "<< planar_cam[2]<<" "<< planar_cam[3]<< std::endl;

        /// save points
        for (size_t j = 0; j < obi.points.size(); ++j) {
            Eigen::Vector4d pt;
            pt << obi.points[j],1;
            pt = Tcl * pt;

            fs_points<< i <<" " << pt[0] <<" "<< pt[1] <<" "<< pt[2]<< std::endl;
        }

        /// save lines
        for (size_t k = 0; k < obi.points_on_line.size(); ++k) {
            Eigen::Vector4d pt;
            pt << obi.points_on_line[k],1;
            pt = Tcl * pt;

            fs_lines<< i <<" " << pt[0] <<" "<< pt[1] <<" "<< pt[2]<< std::endl;
        }
    }

}
#define LOSSFUNCTION
void CamLaserCalibration(const std::vector<Oberserve> obs, Eigen::Matrix4d &Tcl, bool use_linefitting_data)
{
    Eigen::Quaterniond q(Tcl.block<3,3>(0,0));
    double q_coeffs[4] = {q.w(),q.x(),q.y(),q.z()};
    double t_coeffs[3] = {Tcl(0,3),Tcl(1,3),Tcl(2,3)};

    ceres::Problem problem;
    for(size_t i = 0; i< obs.size(); ++i)
    {
        Oberserve obi = obs[i];
        // transform planar in tag frame to cam frame
//        https://stackoverflow.com/questions/7685495/transforming-a-3d-plane-using-a-4x4-matrix
        Eigen::Vector4d planar_tag(0,0,1,0);  // tag 坐标系下的平面方程
        Eigen::Matrix4d Tctag = Eigen::Matrix4d::Identity();
        Tctag.block(0,0,3,3) = obi.tagPose_Qca.toRotationMatrix();
        Tctag.block(0,3,3,1) = obi.tagPose_tca;
        Eigen::Vector4d planar_cam = (Tctag.inverse()).transpose() * planar_tag;

        std::vector<Eigen::Vector3d> calibra_pts;
        if(use_linefitting_data)
            calibra_pts =  obi.points_on_line;
        else
            calibra_pts =  obi.points;

        for (size_t j = 0; j < calibra_pts.size(); ++j) {
            Eigen::Vector3d pt =  calibra_pts[j];
            ceres::CostFunction * costfunction =
                new ceres::AutoDiffCostFunction<CamScanPointsErr,1,4,3>(
                        new CamScanPointsErr(planar_cam , pt));

#ifdef LOSSFUNCTION
            //ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
            ceres::LossFunction * loss_function = new ceres::CauchyLoss(0.5);
            problem.AddResidualBlock(costfunction, loss_function, q_coeffs, t_coeffs);
#else
            problem.AddResidualBlock(costfunction, NULL, q_coeffs, t_coeffs, &scale);
#endif
        }
    }

    ceres::LocalParameterization* quaternionParameterization = new ceres::QuaternionParameterization;
    problem.SetParameterization(q_coeffs,quaternionParameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    std::cout << summary.FullReport() << std::endl;

    q = Eigen::Quaterniond(q_coeffs[0],q_coeffs[1],q_coeffs[2],q_coeffs[3]);

    Tcl.block<3,3>(0,0) = q.toRotationMatrix();
    Tcl.block<3,1>(0,3) << t_coeffs[0],t_coeffs[1],t_coeffs[2];

}
/////////////////////////////////////////////////////////////////
struct LineFittingResidfual {
    LineFittingResidfual(double x, double y)
            : x_(x), y_(y) {}

    template <typename T>
    bool operator()(const T* const m, T* residual) const {
        residual[0] = m[0] * T(x_) + m[1] * T(y_) + T(1.);
        return true;
    }

private:
    // Observations for a sample.
    const double x_;
    const double y_;
};

void LineFittingCeres(const std::vector<Eigen::Vector3d> Points, Eigen::Vector2d & Line)
{
    double line[3] = {Line(0),Line(1)};

    ceres::Problem problem;
    for(size_t i = 0; i< Points.size(); ++i)
    {
        Eigen::Vector3d obi = Points[i];

        ceres::CostFunction * costfunction =
                new ceres::AutoDiffCostFunction<LineFittingResidfual,1,2>(
                        new LineFittingResidfual(obi.x(),obi.y()));

#ifdef LOSSFUNCTION
            //ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
            ceres::LossFunction * loss_function = new ceres::CauchyLoss(0.05);
            problem.AddResidualBlock(costfunction, loss_function, line);
#else
            problem.AddResidualBlock(costfunction, NULL, line);
#endif
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 10;

    ceres::Solver::Summary summary;
    ceres::Solve (options, &problem, & summary);

    Line(0) = line[0];
    Line(1) = line[1];
}