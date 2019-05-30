#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "opencv/cxeigen.hpp"

cv::Mat cvK_;
cv::Mat cvD_;
cv::Mat cv_T_;
namespace sc
{

    void TranScanToPoints(const sensor_msgs::LaserScanConstPtr& scan_in, std::vector<Eigen::Vector3d>& Points)
    {
        // http://wiki.ros.org/laser_geometry
        size_t n_pts = scan_in->ranges.size ();
        Eigen::ArrayXXd ranges (n_pts, 2);
        Eigen::ArrayXXd output (n_pts, 2);
        Eigen::ArrayXXd co_sine_map (n_pts, 2);

//        std::cout << "---------- read scan -----------"<<std::endl;
        for (size_t i = 0; i < n_pts; ++i)
        {
            // std::cout << scan_in->ranges[i]<< " "<<scan_in->angle_min + (double) i * scan_in->angle_increment <<std::endl;
            ranges (i, 0) = (double) scan_in->ranges[i];
            ranges (i, 1) = (double) scan_in->ranges[i];

            co_sine_map (i, 0) = cos (scan_in->angle_min + (double) i * scan_in->angle_increment);
            co_sine_map (i, 1) = sin (scan_in->angle_min + (double) i * scan_in->angle_increment);
        }

        output = ranges * co_sine_map;

        for (size_t i = 0; i < n_pts; ++i) {

            // TODO: range_cutoff threashold
            double range_cutoff = 30.0;
            const float range = scan_in->ranges[i];
            if (range < range_cutoff && range >= scan_in->range_min)
            {
                // std::cout << output (i, 0) << " "<<output (i, 1) <<std::endl;
                Points.push_back(Eigen::Vector3d(output (i, 0), output (i, 1), 0) );
            } else
            {
                Points.push_back(Eigen::Vector3d(1000.0,1000.0, 0) );
            }

        }

    }

    void showScanCallback(const sensor_msgs::LaserScanConstPtr &scan_in, 
                          const sensor_msgs::ImageConstPtr &image_in) {

        std::vector<Eigen::Vector3d> p_l;
        TranScanToPoints(scan_in, p_l);

        Eigen::Matrix4d Tlc;
        Tlc.setZero();

        if(!cv_T_.empty())
            cv::cv2eigen(cv_T_, Tlc);
        else{
            std::cerr <<" You Do not have calibra result Tlc. We use a Identity matrix." << std::endl;
            Tlc.setIdentity();
        }

        Eigen::Matrix3d Rlc = Tlc.block(0,0,3,3);
        Eigen::Vector3d tlc(Tlc(0,3),Tlc(1,3),Tlc(2,3));

        Eigen::Matrix3d Rcl = Rlc.transpose();
        Eigen::Vector3d tcl = - Rcl * tlc;

//        std::cout << " Tlc: \n"<< Tlc<<std::endl;

        std::vector<Eigen::Vector3d> p_c;
        std::vector<cv::KeyPoint> keypoints;

        int n_pts = scan_in->ranges.size();
        for (int i = 0; i < n_pts; i++) {
           // std::cout << p_l[i].transpose() << std::endl;
            p_c.emplace_back(Rcl * p_l[i] + tcl); 
        }

        std::vector< cv::Point2f > pixel;
        std::vector< cv::Point3f> pts;

        for (int i = 0; i < n_pts; i++) {            
            double X = p_c[i].x();
            double Y = p_c[i].y();
            double Z = p_c[i].z();

//            pts.push_back( cv::Point3f(X/Z,Y/Z,1.) );
            pts.push_back( cv::Point3f(X,Y,Z) );

        }
        cv::projectPoints(pts, cv::Vec3d::zeros(), cv::Vec3d::zeros(), cvK_, cvD_, pixel);

        cv::Mat img_src = cv_bridge::toCvShare(image_in, "bgr8")->image;

        for (size_t j = 0; j < pixel.size(); ++j) {
            cv::circle(img_src, pixel[j],1, cv::Scalar(0,255,0),1);
        }
        cv::imshow("show", img_src);
        cv::waitKey(10);
    }

} // sc namespace end

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    std::cout << name <<std::endl;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "show_scan");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    /// ==========================================
    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return 0;
    } else
    {
        std::cout << "READ CONFIG_FILE...\n";
    }

    std::string scan_topic_name;
    std::string img_topic_name;
    std::string bag_path;
    std::string save_path;

    fsSettings["scan_topic_name"] >> scan_topic_name;
    fsSettings["img_topic_name"] >> img_topic_name;
    fsSettings["savePath"] >> save_path;
    fsSettings["bag_path"] >> bag_path;

    cv::FileNode n1 = fsSettings["distortion_parameters"];
    double m_k1 = static_cast<double>(n1["k1"]);
    double m_k2 = static_cast<double>(n1["k2"]);
    double m_p1 = static_cast<double>(n1["p1"]);
    double m_p2 = static_cast<double>(n1["p2"]);
    n1 = fsSettings["projection_parameters"];
    double m_fx = static_cast<double>(n1["fx"]);
    double m_fy = static_cast<double>(n1["fy"]);
    double m_cx = static_cast<double>(n1["cx"]);
    double m_cy = static_cast<double>(n1["cy"]);

    fsSettings.release();

    cvK_ = (cv::Mat_<float>(3, 3) << m_fx, 0.0, m_fy, 0.0, m_cx, m_cy, 0.0, 0.0, 1.0);
    cvD_ = (cv::Mat_<float>(1, 5) << m_k1, m_k2, m_p1, m_p2, 0.);

    std::cout <<"\t"<<scan_topic_name << std::endl;
    std::cout <<"\t"<<img_topic_name << std::endl;
    std::cout <<"\t"<<bag_path << std::endl;

    std::string fn = save_path + "result.yaml";
    fsSettings.open(fn,cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "You should run calibra tool first to generate result.yaml in your savePath." << std::endl;
        return 0;
    }
    fsSettings["extrinsicTlc"] >> cv_T_;
    fsSettings.release();

    std::cout << "END CONFIG_FILE\n" << std::endl;
/// ==========================================

    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, scan_topic_name, 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, img_topic_name, 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), scan_sub, image_sub);
    sync.registerCallback(boost::bind(&sc::showScanCallback, _1, _2));
    std::cout << "start spin.." << std::endl;
    ros::spin();

    return 0;
}
