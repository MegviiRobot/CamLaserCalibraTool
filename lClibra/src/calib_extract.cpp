// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// ROS Messages
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/LaserScan.h>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "opencv/cxeigen.hpp"
#include <fstream>

#include "utilities.h"
#include "selectScanPoints.h"
#include "LaseCamCalCeres.h"
#include "config.h"


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
    ros::init(argc, argv, "LaserCamCalibra");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");
    readParameters(config_file);

    // Load apritag pose
    std::vector < CamPose > tagpose;
    LoadCamPoseFromTxt(savePath + "apriltag_pose.txt",tagpose);
    std::cout << "Load apriltag pose size: " << tagpose.size() <<std::endl;

    std::vector< CamPose >  staticCameraPose;
    GetStaticPose(tagpose,staticCameraPose);

    rosbag::Bag bag_input;
    bag_input.open(bag_path, rosbag::bagmode::Read);
    std::vector<std::string> topics;
	topics.push_back(scan_topic_name);
	rosbag::View views(bag_input, rosbag::TopicQuery(topics));

	assert(staticCameraPose.size() > 5 && "static pose is less 5");

    bool flag = true;
    int idx = 0;
    double start_laserSegment_time = staticCameraPose[idx].start_time;
    double end_laserSegment_time = staticCameraPose[idx].end_time;
    double next_start_laserSegment_time = staticCameraPose[idx].start_time;
    double next_end_laserSegment_time = staticCameraPose[idx].end_time;

    std::vector<cv::Rect> rois;
    std::vector<Oberserve> obs;

    int static_frame_cnt = 0;
    std::vector<Eigen::Vector3d> staticFramesPts;
    for(rosbag::MessageInstance const m: views)
    {

        if (m.getTopic() == scan_topic_name) {
            sensor_msgs::LaserScan::Ptr scan = m.instantiate<sensor_msgs::LaserScan>();
            std::vector<Eigen::Vector3d> Points;
            TranScanToPoints(*scan,Points);

            double timestamp = scan->header.stamp.toSec();
            // 按时间段挑选静止时刻的感兴趣的激光数据，对于同一个时间段，只要挑一次便会记住感兴趣区域 rois
            if (timestamp > next_start_laserSegment_time && timestamp< next_end_laserSegment_time)
            {
                rois = PointsToImg(Points);
                // 切换成下一段感兴趣的时间段
                idx ++;
                start_laserSegment_time = next_start_laserSegment_time;
                end_laserSegment_time = next_end_laserSegment_time;
                if(idx < staticCameraPose.size())
                {
                    next_start_laserSegment_time = staticCameraPose[idx].start_time;
                    next_end_laserSegment_time = staticCameraPose[idx].end_time;
                } else
                {
                    next_start_laserSegment_time = 1e18;
                    next_end_laserSegment_time = -1.0;
                }

            }

            // 把静止时间段内的所有激光数据数据记录下来，拟合一条直线，然后取直线上的任意两个点作为3d点用于标定外参数
            if(timestamp > start_laserSegment_time && timestamp < end_laserSegment_time)
            {

                int index = idx - 1;   // 上面idx已经跳到下一个静止时间段了，但是我们保存的还是这个静止时间段的激光数据
                std::vector<Eigen::Vector3d> points;
                points = GetROIScanPoints(Points,rois);

                // 至少超过 50 个点， 选取的激光线段至少超过 15 cm
//                if(points.size() > 50 && (points[0] - points[points.size()-1]).norm() > 0.15)
                if((points[0] - points[points.size()-1]).norm() > 0.15)
                {
                    if(static_frame_cnt == 0)  // new static segment
                    {
                        staticFramesPts = points;
                        static_frame_cnt ++;
                    }else
                    {
                        // 把这段时间内的所有点都保存起来
                        for (size_t k = 0; k < points.size(); ++k) {
                                staticFramesPts.push_back( points[k]);
                            }
                    }

                } else
                {
                    std::cout << "Discard frame, valid scan points num: "<< points.size() << std::endl;
                }
            } else
            {
                if(static_frame_cnt > 0)
                {

                    // 用这一堆数据，拟合一条直线，之前的约束假设他们是在平面上，约束还不够强
                    double x_min(1000.0), x_max(-1000.), y_min(1000.), y_max(-1000.);
                    for (int i = 0; i < staticFramesPts.size(); ++i) {
                        Eigen::Vector3d pt = staticFramesPts[i];
                        if(pt.x() < x_min)
                            x_min = pt.x();
                        if(pt.x() > x_max)
                            x_max = pt.x();

                        if(pt.y() < y_min)
                            y_min = pt.y();
                        if(pt.y() > y_max)
                            y_max = pt.y();
                    }
                    Eigen::Vector2d line;
                    LineFittingCeres(staticFramesPts,line);
                    std::vector<Eigen::Vector3d> points_on_line;

                    if(x_max - x_min > y_max - y_min)
                    {
                        y_min = - (x_min * line(0) + 1) / line(1);
                        y_max = - (x_max * line(0) + 1) / line(1);

                    } else // 垂直于 x 轴
                    {
                        x_min = - (y_min * line(1) + 1) / line(0);
                        x_max = - (y_max * line(1) + 1) / line(0);
                    }

                    points_on_line.push_back(Eigen::Vector3d(x_min,y_min,0));
                    points_on_line.push_back(Eigen::Vector3d(x_max,y_max,0));

                    Oberserve ob;
                    CamPose T = staticCameraPose[idx - 1];
                    ob.tagPose_Qca = T.qwc;
                    ob.tagPose_tca = T.twc;
                    ob.points = staticFramesPts;
                    ob.points_on_line = points_on_line;
                    obs.push_back(ob);
                }
                static_frame_cnt = 0;
            }

        }
    }

    if(obs.size() < 5)
    {
        std::cout << "Valid Calibra Data Less"<<std::endl;
        bag_input.close();
        return 0;
    }

    Eigen::Matrix4d Tcl = Eigen::Matrix4d::Identity();
    CamLaserCalibration(obs,Tcl,false);

    std::cout << "\n----- Transform from Camera to Laser Tlc is: -----\n"<<std::endl;
    std::cout<<Tcl.inverse()<<std::endl;

    // save to yaml file
    cv::Mat cvTlc;
    Eigen::Matrix4d Tlc = Tcl.inverse();
    cv::eigen2cv(Tlc,cvTlc);
    std::string fn = savePath + "result.yaml";
    cv::FileStorage fs(fn, cv::FileStorage::WRITE);
    fs << "extrinsicTlc"<<cvTlc;
    fs.release();
    std::cout << "\n Result file : "<<fn<<std::endl;
    std::cout << "\n-------------- Calibration Code End --------------\n"<<std::endl;

    CalibrationTool_SavePlanePoints(obs,Tcl,savePath);
//    bag_input.close();

    return 1;
}
