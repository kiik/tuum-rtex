
#ifndef RTX_BLOBBER_H
#define RTX_BLOBBER_H

#include <vector>

#include <opencv2/opencv.hpp>

#include "cmv/blob_finder.hpp"

namespace rtx {

  class GameField;

  typedef std::vector<cmv::blob_t> BlobSet;

  int object_detection_init();
  int object_detection(cv::Mat&, GameField*);

}

#endif
