//
// Created by heyijia on 18-12-11.
//
#include "utilities.h"

void LoadCamPoseFromTxt(std::string filename, std::vector < CamPose >& allcampose )
{
    std::ifstream f;
    f.open(filename.c_str());

    if(!f.is_open())
    {
        std::cerr << " can't open cam pose file "<<std::endl;
        return;
    } else
    {
        std::cout << " Load apriltag pose from: "<<filename <<std::endl;
    }

    while (!f.eof()) {

        std::string s;
        std::getline(f,s);

        if(! s.empty())
        {
            std::stringstream ss;
            ss << s;
            double  timestamp;
            ss >> timestamp;

            double x,y,z,q1,q2,q3,qw;
            ss >> x;
            ss >> y;
            ss >> z;
            ss >> q1;
            ss >> q2;
            ss >> q3;
            ss >> qw;

            Eigen::Quaterniond qua(qw, q1,q2,q3);

            CamPose cp;
            cp.qwc = qua;
            cp.twc = Eigen::Vector3d(x,y,z);
            cp.timestamp = timestamp;

            allcampose.push_back(cp);

            //list_T_w_r_.push_back(std::make_pair(timestamp,T));

        }
    }
}

void SaveCamPosetoTxt(std::string filename, const std::vector < CamPose > allcampose )
{
    std::ofstream foutC;
    foutC.open(filename.c_str());

    if(!foutC.is_open())
    {
        std::cerr << " can't open cam pose file "<<std::endl;
        return;
    }

    for (size_t i = 0; i < allcampose.size(); ++i) {
        CamPose T = allcampose.at(i);
        foutC.setf(std::ios::fixed, std::ios::floatfield);
        foutC.precision(9);
        foutC << T.timestamp << " ";
        foutC.precision(5);
        foutC << T.twc.x() << " "
              << T.twc.y() << " "
              << T.twc.z() << " "
              << T.qwc.x() << " "
              << T.qwc.y() << " "
              << T.qwc.z() << " "
              << T.qwc.w() <<std::endl;
    }
    foutC.close();
}


// 从一系列连续的轨迹中，获取静止的那些 pose 点
std::vector< std::vector < CamPose > >  GetStaticPose(const std::vector < CamPose > Poses, std::vector< CamPose > &avergeStaticPoses)
{
    assert(Poses.size() != 0 && "NO POSE ENTRY");
    Eigen::Vector3d xy_sum(0,0,0);
    bool new_center_flag = true;
    Eigen::Vector3d center(0.,0.,0.);
    std::vector<CamPose> staticPose;

    std::vector< std::vector < CamPose > > staticPoses;
    // 按时间顺序遍历所有的 pose
    for (size_t i = 0; i < Poses.size(); ++i) {
        CamPose T = Poses[i];
        if(new_center_flag)
        {
            new_center_flag = false;
            xy_sum = T.twc;
            center = T.twc;
            staticPose.push_back(T);
        }

        Eigen::Vector3d diff = T.twc - center;
        // 当前pose和之前的pose中心相差不远就加入静止点
        if(diff.norm() < 0.002)
        {
            xy_sum += T.twc;
            staticPose.push_back(T);
            center = xy_sum/staticPose.size();

        }else{
            new_center_flag = true;
            xy_sum = Eigen::Vector3d::Zero();

            // 静止了一段时间
            if(staticPose.size() > 30)
                staticPoses.push_back(staticPose);

            staticPose.clear();
        }
    }

    // save average pose
    // https://github.com/fizyr/dr_eigen/blob/master/include/dr_eigen/average.hpp
    // https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
    for (size_t i = 0; i < staticPoses.size(); ++i) {

        CamPose c;
        c.start_time = staticPoses[i].front().timestamp;
        c.end_time = staticPoses[i].back().timestamp;

        Eigen::Vector3d c_t_mea(0.0,0.,0.);
        Eigen::Quaterniond c_q_mean(0,0,0,1);
        Eigen::Matrix4d A_c = Eigen::Matrix4d::Zero();

        size_t n =  staticPoses[i].size();
        for (size_t j = 0; j < n; ++j) {
            c_t_mea += staticPoses[i][j].twc;
            Eigen::Vector4d qc(staticPoses[i][j].qwc.w(),staticPoses[i][j].qwc.x(),staticPoses[i][j].qwc.y(),staticPoses[i][j].qwc.z());
            A_c += qc * qc.transpose();
        }
        c.twc = c_t_mea / n;
        A_c /= n;
        c.qwc = getAvergeQwc(A_c);
//        std::cout << "----------------------------------\n";
//        std::cout << c.twc.transpose()<<std::endl<< staticPoses[i][(int)n/2].twc.transpose()<< std::endl;
//        std::cout << c.qwc.coeffs().transpose()<<std::endl<< staticPoses[i][(int)n/2].qwc.coeffs().transpose()<< std::endl;
        avergeStaticPoses.push_back(c);
    }
    return staticPoses;

}
Eigen::Quaterniond getAvergeQwc(Eigen::Matrix4d A)
{
    Eigen::EigenSolver<Eigen::Matrix<double, 4, 4>> es(A);

    Eigen::Matrix<std::complex<double>, 4, 1> mat(es.eigenvalues());
    int index;
    mat.real().maxCoeff(&index);
    Eigen::Matrix<double, 4, 1> largest_ev(es.eigenvectors().real().block(0, index, 4, 1));

    return Eigen::Quaternion<double>(largest_ev(0), largest_ev(1), largest_ev(2), largest_ev(3));
}

std::vector < CamPose > GetInversePose(std::vector < CamPose > pose)
{
    std::vector < CamPose > invpose;
    for (auto T : pose) {
        CamPose invT;
        invT = T;
        invT.twc = -T.qwc.toRotationMatrix().transpose() * T.twc;
        invT.qwc = T.qwc.inverse();
        invpose.push_back(invT);
    }
    return invpose;
}

void TranScanToPoints(const sensor_msgs::LaserScan& scan_in,std::vector<Eigen::Vector3d>& Points)
{
    // http://wiki.ros.org/laser_geometry
    size_t n_pts = scan_in.ranges.size ();
    Eigen::ArrayXXd ranges (n_pts, 2);
    Eigen::ArrayXXd output (n_pts, 2);
    Eigen::ArrayXXd co_sine_map (n_pts, 2);

    for (size_t i = 0; i < n_pts; ++i)
    {
        ranges (i, 0) = (double) scan_in.ranges[i];
        ranges (i, 1) = (double) scan_in.ranges[i];

        co_sine_map (i, 0) = cos (scan_in.angle_min + (double) i * scan_in.angle_increment);
        co_sine_map (i, 1) = sin (scan_in.angle_min + (double) i * scan_in.angle_increment);
    }

    output = ranges * co_sine_map;

    for (size_t i = 0; i < n_pts; ++i) {

        // TODO: range_cutoff threashold
        double range_cutoff = 30.0;
        const float range = scan_in.ranges[i];
        if (range < range_cutoff && range >= scan_in.range_min)
        {
            Points.push_back(Eigen::Vector3d(output (i, 0), output (i, 1), 0) );
        } else
        {
            Points.push_back(Eigen::Vector3d(1000.0,1000.0, 0) );
        }

    }

}

Eigen::Quaterniond yprToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Eigen::Quaterniond q;
    q.w() = cy * cp * cr + sy * sp * sr;
    q.x() = cy * cp * sr - sy * sp * cr;
    q.y() = sy * cp * sr + cy * sp * cr;
    q.z() = sy * cp * cr - cy * sp * sr;

    return q;
}

EulerAngles ToEulerAngles(Eigen::Quaterniond q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}


/*
 三点确定一个平面 a(x-x0)+b(y-y0)+c(z-z0)=0  --> ax + by + cz + d = 0   d = -(ax0 + by0 + cz0)
 平面通过点（x0,y0,z0）以及垂直于平面的法线（a,b,c）来得到
 (a,b,c)^T = vector(AO) cross vector(BO)
 d = O.dot(cross(AO,BO))
 */
Eigen::Vector4d pi_from_ppp(Eigen::Vector3d x1, Eigen::Vector3d x2, Eigen::Vector3d x3) {
    Eigen::Vector4d pi;
    pi << ( x1 - x3 ).cross( x2 - x3 ), - x3.dot( x1.cross( x2 ) ); // d = - x3.dot( (x1-x3).cross( x2-x3 ) ) = - x3.dot( x1.cross( x2 ) )

    return pi;
}

/*
 * 返回直线相交点
 */
bool lineCrossPt(const StraightLine& l1, const StraightLine& l2, Eigen::Vector2d& pt)
{
    Eigen::Vector2d A = l1.A;
    Eigen::Vector2d B = l1.B;

    Eigen::Vector2d C = l2.A;
    Eigen::Vector2d D = l2.B;

    double d = (A.x() - B.x()) * (C.y() - D.y()) - (A.y() - B.y()) * (C.x() - D.x());
    if(std::abs(d) < 1e-10)   // 两者接近平行
    {
        return false;
    }
    double x1y2_y1x2 = A.x() * B.y() - A.y() * B.x();
    double x3y4_y3x4 = C.x() * D.y() - C.y() * D.x();
    double x3_x4 = C.x() - D.x();
    double y3_y4 = C.y() - D.y();
    double x1_x2 = A.x() - B.x();
    double y1_y2 = A.y() - B.y();

    pt = 1./d * Eigen::Vector2d(x1y2_y1x2 * x3_x4 - x1_x2 * x3y4_y3x4, x1y2_y1x2 *  y3_y4 - y1_y2 * x3y4_y3x4);

    double x_min = std::min( l1.A.x(), l1.B.x());
    double x_max = std::max( l1.A.x(), l1.B.x());
    double y_min = std::min( l1.A.y(), l1.B.y());
    double y_max = std::max( l1.A.y(), l1.B.y());

//    // 不在线段内
//    if(pt.x() < x_min - 1e-6 || pt.x() > x_max + 1e-6 || pt.y() < y_min - 1e-6 ||  pt.y() > y_max + 1e-6)
//    {
////        std::cout << " not in line seg L1" <<std::endl;
////        std::cout << "        " <<pt.x()<<" " << pt.y() <<std::endl;
//        return false;
//    }
//
//
//    x_min = std::min( l2.A.x(), l2.B.x());
//    x_max = std::max( l2.A.x(), l2.B.x());
//    y_min = std::min( l2.A.y(), l2.B.y());
//    y_max = std::max( l2.A.y(), l2.B.y());
//    // 不在线段内
//    if(pt.x() < x_min - 1e-6 || pt.x() > x_max + 1e-6 || pt.y() < y_min - 1e-6 ||  pt.y() > y_max + 1e-6)
//    {
////        std::cout << " not in line seg L2" <<std::endl;
//        return false;
//    }

    return true;
}
