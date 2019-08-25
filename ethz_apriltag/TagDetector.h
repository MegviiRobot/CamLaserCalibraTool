#ifndef TAGDETECTOR_H
#define TAGDETECTOR_H

#include <vector>

#include "opencv2/opencv.hpp"

#include "FloatImage.h"
#include "TagDetection.h"
#include "TagFamily.h"

namespace AprilTags {

class TagDetector {
 public:
  const TagFamily thisTagFamily;

  //! Constructor
  // note: TagFamily is instantiated here from TagCodes
  TagDetector(const TagCodes &tagCodes, const size_t blackBorder = 2)
      : thisTagFamily(tagCodes, blackBorder) {}

  std::vector<TagDetection> extractTags(const cv::Mat &image);
};

}  // namespace AprilTags

#endif
