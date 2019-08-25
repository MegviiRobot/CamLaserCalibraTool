#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "Camera.h"
#include "Tag36h11.h"
#include "TagDetector.h"

enum PatternType
{
    KALIBR_TAG_PATTERN = 1,
    APRIL_TAG_ONE = 2,
    CHESS = 3,
    CIRCLE = 4
};

struct CalibrBoardInfo
{
public:
    CalibrBoardInfo()
    {
        pt_ = KALIBR_TAG_PATTERN;
        tagSize_ = 0.055;
        rows_ = 6;
        cols_ = 6;
        black_border_ = 2;
    };

    CalibrBoardInfo(PatternType pt, double tagsize, double tagspacing = 0.3,int black_border = 1,int rows = 6, int cols = 6):
            pt_(pt),
            tagSize_(tagsize),
            tagSpacing_(tagspacing),
            black_border_(black_border),
            rows_(rows),
            cols_(cols)
         {};
    PatternType pt_;
    double tagSize_;
    double tagSpacing_;
    int rows_;
    int cols_;
    int black_border_;
};

class CamPoseEst
{
public:
    CamPoseEst(){};
    CamPoseEst(CalibrBoardInfo &info):CalBoardInfo_(info){};

    void FindTargetCorner(cv::Mat &img_raw,
                          std::vector<cv::Point3f> &p3ds,
                          std::vector<cv::Point2f> &p2ds);

    bool EstimatePose(const std::vector<cv::Point3f> &p3ds,
                      const std::vector<cv::Point2f> &p2ds,
                      const CameraPtr &cam,
                      Eigen::Matrix4d &Twc, cv::Mat &img_raw);

    bool calcCamPose(const double &timestamps, const cv::Mat &image,
                     const CameraPtr &cam, Eigen::Matrix4d &Twc);

    CalibrBoardInfo CalBoardInfo_;

};
