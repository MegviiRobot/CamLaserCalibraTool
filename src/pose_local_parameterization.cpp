#include "pose_local_parameterization.h"

Eigen::Quaterniond deltaQ(const Eigen::Vector3d &theta)
{
    Eigen::Quaterniond dq;
    Eigen::Vector3d half_theta = theta;
    half_theta /= static_cast<double>(2.0);
    dq.w() = static_cast<double>(1.0);
    dq.x() = half_theta.x();
    dq.y() = half_theta.y();
    dq.z() = half_theta.z();
    return dq;
}

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
    Eigen::Map<const Eigen::Vector3d> _p(x);
    Eigen::Map<const Eigen::Quaterniond> _q(x + 3);

    Eigen::Map<const Eigen::Vector3d> dp(delta);

    Eigen::Quaterniond dq = deltaQ(Eigen::Map<const Eigen::Vector3d>(delta + 3));

    Eigen::Map<Eigen::Vector3d> p(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta + 3);

    p = _p + dp;
    q = (_q * dq).normalized();

    return true;
}

bool PoseLocalParameterization::Minus(const double *y, const double *x, double *y_minus_x) const
{
    Eigen::Map<const Eigen::Vector3d> p_x(x);
    Eigen::Map<const Eigen::Quaterniond> q_x(x + 3);

    Eigen::Map<const Eigen::Vector3d> p_y(y);
    Eigen::Map<const Eigen::Quaterniond> q_y(y + 3);

    Eigen::Map<Eigen::Matrix<double, 6, 1>> delta(y_minus_x);
    delta.head<3>() = p_y - p_x;

    const Eigen::Quaterniond dq = q_x.conjugate() * q_y;
    delta.tail<3>() = 2.0 * Eigen::Vector3d(dq.x(), dq.y(), dq.z());
    return true;
}

bool PoseLocalParameterization::PlusJacobian(const double *x, double *jacobian) const
{
    (void)x;
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
    j.topRows<6>().setIdentity();
    j.bottomRows<1>().setZero();

    return true;
}

bool PoseLocalParameterization::MinusJacobian(const double *x, double *jacobian) const
{
    (void)x;
    Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>> j(jacobian);
    j.setZero();
    j.leftCols<6>().setIdentity();
    return true;
}
