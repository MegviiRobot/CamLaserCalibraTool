// #include <algorithm>
// #include <cmath>
#include <opencv2/core/core.hpp>

template <class T>
const T square(const T &x)
{
    return x * x;
}

void fitCircle(const std::vector<cv::Point2d> &points, double &centerX, double &centerY, double &radius);

std::vector<cv::Point2d> intersectCircles(double x1, double y1, double r1,
                                          double x2, double y2, double r2);