#include "calcCamPose.h"
#include "tic_toc.h"

//std::string file1= "/home/heyijia/ros_ws/apriltag_ws/src/log/kalibra.txt";
void CamPoseEst::FindTargetCorner(cv::Mat &img_raw,
                      std::vector<cv::Point3f> &p3ds,
                      std::vector<cv::Point2f> &p2ds)
{
  // 不同 TAG 类型，使用不同姿态求解
  if (CHESS == CalBoardInfo_.pt_)
  {
    // std::cout << "CHESSBOARD\n";
    const int col = CalBoardInfo_.cols_;    // 9
    const int row = CalBoardInfo_.rows_;    // 6
    const float square_size = CalBoardInfo_.tagSize_;        // 0.575 unit:  m
    cv::Size pattern_size(col, row);
    std::vector<cv::Point2f> corners;
    if (cv::findChessboardCorners(img_raw, pattern_size, corners))
    {
      cv::cornerSubPix(
          img_raw, corners, cv::Size(11, 11), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      if (corners.size() == col * row)
      {
        int count = 0;
        for (int i = 0; i < row; i++)
        {
          for (int j = 0; j < col; j++)
          {
            // Todo: change 3d-coordinate
            p3ds.emplace_back(
                cv::Point3f(j * square_size, i * square_size, 0.0));
            p2ds.emplace_back(cv::Point2f(corners[count].x, corners[count].y));
            count++;
          }
        }
      }
      else
      {
        std::cout << "Chessboard config is not correct with image\n";
      }
    }
    else
    {
      std::cout << "No chessboard detected in image\n";
    }
  }
  else if (KALIBR_TAG_PATTERN == CalBoardInfo_.pt_)
  {
    const int april_rows = CalBoardInfo_.rows_;      // 6
    const int april_cols = CalBoardInfo_.cols_;      // 6
    const double tag_sz = CalBoardInfo_.tagSize_;    // 0.055
    const double tag_spacing = CalBoardInfo_.tagSpacing_;    // 0.3
    const double tag_spacing_sz = tag_sz * (1.+tag_spacing); // 0.055 + 0.0165

    std::cout <<"tag size: "<< tag_sz <<" tag space: "<< tag_spacing  << std::endl;
    AprilTags::TagCodes tagCodes(AprilTags::tagCodes36h11);
    AprilTags::TagDetector detector(tagCodes, CalBoardInfo_.black_border_);

    TicToc tcnt;
    std::vector<AprilTags::TagDetection> detections = detector.extractTags(img_raw);
    double timecost =  tcnt.toc();

//    std::cout << timecost << std::endl;
//    std::ofstream foutC_1(file1.c_str(), std::ios::app);
//    foutC_1.setf(std::ios::fixed, std::ios::floatfield);
//    foutC_1.precision(5);
//    foutC_1<<timecost<<" "<<detections.size()<<std::endl;
//    foutC_1.close();123

//    if (detections.size() == april_rows * april_cols)
    if (detections.size() > 0)
    {
      std::sort(detections.begin(), detections.end(),
                AprilTags::TagDetection::sortByIdCompare);
      cv::Mat tag_corners(4 * detections.size(), 2, CV_32F);
      for (unsigned i = 0; i < detections.size(); i++)
      {
        for (unsigned j = 0; j < 4; j++)
        {
          tag_corners.at<float>(4 * i + j, 0) = detections[i].p[j].first;
          tag_corners.at<float>(4 * i + j, 1) = detections[i].p[j].second;
        }
      }

      int window_size = 3;
      cv::cornerSubPix(
          img_raw, tag_corners, cv::Size(window_size, window_size), cv::Size(-1, -1),
          cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
      cv::cvtColor(img_raw, img_raw, CV_GRAY2BGR);

      int index = 0;
      for (auto &tag : detections)
      {
          // debug show corners with window size
          {
              for (int i = 0; i < 4; ++i) {
                  cv::Point2f cor1(tag.p[i].first - window_size,tag.p[i].second-window_size);
                  cv::Point2f cor2(tag.p[i].first + window_size,tag.p[i].second+window_size);
                  cv::rectangle(img_raw, cor1,cor2, cv::Scalar(0, 255, 0),1);
              }
          }

        unsigned int id = tag.id;
        unsigned int tag_row = id / april_cols;
        unsigned int tag_col = id % april_cols;

        cv::circle(img_raw, cv::Point2f(tag.cxy.first, tag.cxy.second), 3,
                   cv::Scalar(0, 255, 0));
        cv::putText(img_raw, std::to_string(id),
                    cv::Point2f(tag.cxy.first, tag.cxy.second),
                    CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

          // 四个角点坐标
        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col,
                                      tag_spacing_sz * tag_row, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        ++index;

        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col + tag_sz,
                                      tag_spacing_sz * tag_row, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        ++index;

        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col + tag_sz,
                                      tag_spacing_sz * tag_row + tag_sz, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        ++index;

        p3ds.emplace_back(cv::Point3f(tag_spacing_sz * tag_col,
                                      tag_spacing_sz * tag_row + tag_sz, 0.0));
        p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(index, 0),
                                      tag_corners.at<float>(index, 1)));
        ++index;
      }
    }
  }
  else if(APRIL_TAG_ONE == CalBoardInfo_.pt_)  // TODO: 只有一个 April tag
  {

      const double tag_sz = CalBoardInfo_.tagSize_;    // 0.165 m

      AprilTags::TagCodes tagCodes(AprilTags::tagCodes36h11);
      AprilTags::TagDetector detector(tagCodes, CalBoardInfo_.black_border_);

      std::vector<AprilTags::TagDetection> detections = detector.extractTags(img_raw);

      if(detections.size() > 0)
      {
          if(detections.size() > 1)
          {
              std::cout << "uhhhhh, too many apritags!!!\n";
              return;
          }

          AprilTags::TagDetection tag = detections[0];
          cv::Mat tag_corners(4 , 2, CV_32F);

          for (unsigned j = 0; j < 4; j++)
          {
              tag_corners.at<float>(j, 0) = tag.p[j].first;
              tag_corners.at<float>(j, 1) = tag.p[j].second;
          }

          // 亚像素
//          int window_size = 3;
//          cv::cornerSubPix(
//                  img_raw, tag_corners, cv::Size(window_size, window_size), cv::Size(-1, -1),
//                  cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

          cv::cvtColor(img_raw, img_raw, CV_GRAY2BGR);

          cv::circle(img_raw, cv::Point2f(tag.cxy.first, tag.cxy.second), 3,
                     cv::Scalar(0, 255, 0));
          cv::putText(img_raw, std::to_string(tag.id),
                      cv::Point2f(tag.cxy.first, tag.cxy.second),
                      CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

          // 四个角点坐标
          p3ds.emplace_back(cv::Point3f(0., 0., 0.0));
          p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(0, 0),
                                        tag_corners.at<float>(0, 1)));

          p3ds.emplace_back(cv::Point3f(tag_sz, 0., 0.0));
          p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(1, 0),
                                        tag_corners.at<float>(1, 1)));

          p3ds.emplace_back(cv::Point3f(tag_sz, tag_sz, 0.0));
          p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(2, 0),
                                        tag_corners.at<float>(2, 1)));

          p3ds.emplace_back(cv::Point3f(0., tag_sz, 0.0));
          p2ds.emplace_back(cv::Point2f(tag_corners.at<float>(3, 0),
                                        tag_corners.at<float>(3, 1)));

      } else
      {
          std::cout << "Do not detect any apritag!!!\n";
      }

  }
  else
  {
    std::cout << "Pattern type not supported yet.\n";
  }
}

bool CamPoseEst::EstimatePose(const std::vector<cv::Point3f> &p3ds,
                  const std::vector<cv::Point2f> &p2ds,
                  const CameraPtr &cam,
                  Eigen::Matrix4d &Twc, cv::Mat &img_raw)
{
  Twc.setIdentity();
  if (p3ds.size() != p2ds.size() || p3ds.size() < 4)
  {
    return false;
  }

  cv::Mat_<float> K = (cv::Mat_<float>(3, 3) << 1, 0.0, 0, 0.0, 1, 0, 0.0, 0.0, 1.0);
  cv::Mat_<float> dist = (cv::Mat_<float>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
  cv::Mat cv_r, cv_t;
  cv::Mat inliers;
  cv::solvePnP(p3ds, p2ds, K, dist, cv_r, cv_t);
//  cv::solvePnPRansac(p3ds, p2ds, K, dist, cv_r, cv_t, false, 100, 2., 0.99);
  cv::Mat rotation;
  cv::Rodrigues(cv_r, rotation);
  Eigen::Matrix3d Rcw;
  cv::cv2eigen(rotation, Rcw);
  Eigen::Vector3d tcw;
  cv::cv2eigen(cv_t, tcw);
  Twc.block<3, 3>(0, 0) = Rcw.inverse();
  Twc.block<3, 1>(0, 3) = -Rcw.inverse() * tcw;

  std::cout << "twc: "<<Twc(0, 3)<<" "<<Twc(1, 3)<<" "<<Twc(2, 3)<<std::endl;
  cv::putText(
      img_raw, "t_wc: (m) " + std::to_string(Twc(0, 3)) + " " + std::to_string(Twc(1, 3)) + " " + std::to_string(Twc(2, 3)),
      cv::Point2f(50, 30), CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));

//  std::vector<cv::Point3f> axis;
//  std::vector<cv::Point2f> imgpts;
//  axis.push_back(cv::Point3f(0,0,0));
//  axis.push_back(cv::Point3f(1,0,0));
//  axis.push_back(cv::Point3f(0,1,0));
//  axis.push_back(cv::Point3f(0,0,1));
//  cv::projectPoints(axis, cv_r, cv_t, K, dist,imgpts);

  std::vector<Eigen::Vector3d> axis;
  std::vector<cv::Point2f> imgpts;
  axis.push_back(Eigen::Vector3d(0,0,0));
  axis.push_back(Eigen::Vector3d(0.2,0,0));
  axis.push_back(Eigen::Vector3d(0,0.2,0));
  axis.push_back(Eigen::Vector3d(0,0,0.2));
  for (size_t i = 0; i < axis.size(); ++i) {
    Eigen::Vector2d pt;
    Eigen::Vector3d Pt = Rcw * axis[i] + tcw;
    cam->spaceToPlane(Pt, pt);   // 三维空间点，加上畸变投影到图像平面
    imgpts.push_back(cv::Point2f(pt.x(),pt.y()));
  }

  cv::line(img_raw, imgpts[0], imgpts[1], cv::Scalar(255,0,0), 2);
  cv::line(img_raw, imgpts[0], imgpts[2], cv::Scalar(0,255,0), 2);
  cv::line(img_raw, imgpts[0], imgpts[3], cv::Scalar(0,0,255), 2);

  return true;
}

bool CamPoseEst::calcCamPose(const double &timestamps, const cv::Mat &image,
                 const CameraPtr &cam, Eigen::Matrix4d &Twc)
{
  cv::Mat img_raw = image.clone();
  if (img_raw.channels() == 3)
  {
    cv::cvtColor(img_raw, img_raw, CV_BGR2GRAY);
  }

  std::vector<cv::Point3f> p3ds;
  std::vector<cv::Point2f> p2ds;
  // FindTargetCorner(img_raw, CHESS, p3ds, p2ds);
  FindTargetCorner(img_raw, p3ds, p2ds);

  std::vector<cv::Point2f> un_pts;
  for (int i = 0, iend = (int)p2ds.size(); i < iend; ++i)
  {
    Eigen::Vector2d a(p2ds[i].x, p2ds[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);     // 去图像畸变， 得到归一化图像平面的坐标
    un_pts.push_back(
            cv::Point2f(b.x() / b.z(), b.y() / b.z() ));
  }

  if (EstimatePose(p3ds, un_pts, cam, Twc, img_raw))
  {
     cv::imshow("apriltag_detection & camPose_calculation", img_raw);
     cv::waitKey(1);
    return true;
  }
  else
  {
    return false;
  }
}
