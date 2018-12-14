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
        if(diff.norm() < 0.005)
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
