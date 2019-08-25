//
// Created by heyijia on 18-12-13.
//

#include "config.h"
#include <opencv2/opencv.hpp>

std::string savePath = "/home/heyijia/1TDisk/calibra/";
std::string bag_path = "2018-12-13-11-40-16.bag";

std::string scan_topic_name = "/scan";
std::string img_topic_name = "/mynteye/left/image_raw";


void readParameters(std::string config_file)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    fsSettings["scan_topic_name"] >> scan_topic_name;
    fsSettings["img_topic_name"] >> img_topic_name;
    fsSettings["savePath"] >> savePath;
    fsSettings["bag_path"] >> bag_path;

    fsSettings.release();
}