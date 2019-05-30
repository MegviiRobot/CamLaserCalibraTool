//
// Created by heyijia on 18-12-11.
//
#include "selectScanPoints.h"

int img_w = 720;
double focal = 450;
double z = 10;    // 在 10m 高处装一个相机，咔咔给激光点云拍照

std::vector<cv::Rect> PointsToImg(const std::vector<Eigen::Vector3d> points, bool selectPoint)
{

    cv::Mat img(img_w, img_w, CV_8UC1, cv::Scalar::all(0));

    for (auto pt: points) {
        int col = (int)(pt.x() / z * focal + img_w/2);
        int row = (int)(- pt.y() / z * focal + img_w/2);  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果

        if(col > img_w-1 || col< 0 || row > img_w-1 || row < 0)
            continue;

        img.at<uchar>(row, col) = 255;
    }

    std::vector<cv::Rect> rects;
    if(selectPoint)
    {
        GetRect getrect;
        getrect.gettingROI(img);  // 选取感兴趣的图像区域
        rects = getrect.rects;
    } else
    {
        cv::imshow("ScanPoint",img);
        cv::waitKey(0);
    }

    return rects;

}

std::vector<Eigen::Vector3d> GetROIScanPoints(const std::vector<Eigen::Vector3d> points, const std::vector<cv::Rect> rects)
{
    std::vector<Eigen::Vector3d> RoiPoints;
    for (auto rect : rects) {

        for (auto pt: points)
        {
            // 将点转换成图像坐标，然后看是否落在感兴趣区域内部
            int col = (int)(pt.x() / z * focal + img_w/2);
            int row = (int)(- pt.y() / z * focal + img_w/2);  // -Y/Z 加了一个负号, 是为了抵消针孔投影时的倒影效果
            if(col > rect.x + rect.width || col< rect.x || row > rect.y +rect.height || row < rect.y)
                continue;
            RoiPoints.push_back(pt);
        }
    }
    return RoiPoints;
}

void GetRect::onMouse(int event, int x, int y, int flags, void *userdata) {
    // Check for null pointer in userdata and handle the error
    GetRect* temp = reinterpret_cast<GetRect*>(userdata);
    temp->CallBackFunc(event, x, y);

}
void GetRect::CallBackFunc(int event, int x, int y) {

    if ( event == cv::EVENT_LBUTTONDOWN )
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;

        // Init your rect
        base.x = x;
        base.y = y;
        r.x = x;
        r.y = y;
        r.width = 0;
        r.height = 0;
        bDraw = true;
    }
    else if ( event == cv::EVENT_MOUSEMOVE )
    {
//        std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;

        // If drawing, update rect width and height
        if(!bDraw) return;

        int dx = abs(r.x - x);
        int dy = abs(r.y - y);

        if(x < base.x) {
            r.x = x;
            r.width = abs(x - base.x);
        } else {
            r.width = dx;
        }

        if(y < base.y) {
            r.y = y;
            r.height = abs(y - base.y);
        } else {
            r.height = dy;
        }

        // Refresh
        working = layer.clone();
        cv::rectangle(working, r, cv::Scalar(125));
        cv::imshow("ScanPoint", working);
    }
    else if ( event == cv::EVENT_LBUTTONUP)
    {
        std::cout << "Left button released" << std::endl;

        // Save rect, draw it on layer
        rects.push_back(r);
        cv::rectangle(layer, r, cv::Scalar(125));

        r = cv::Rect();
        bDraw = false;

        // Refresh
        working = layer.clone();
        cv::rectangle(working, r, cv::Scalar(125));
        cv::imshow("ScanPoint", working);
    }
}
void GetRect::gettingROI(cv::Mat img)
{
    layer = img.clone();
    cv::putText(layer, "Please Select ROI scan data with your mouse. Then, Put any key to continue!",
                cvPoint(5,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cvScalar(255), 1, CV_AA);
    cv::namedWindow("ScanPoint", 1);
    cv::imshow("ScanPoint", layer);
    cv::setMouseCallback("ScanPoint", GetRect::onMouse, this);
    cv::waitKey(0);
}
