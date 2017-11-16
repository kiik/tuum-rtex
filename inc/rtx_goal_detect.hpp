
#include <vector>

#include <aruco/aruco.h>

namespace rtx {

  typedef std::vector<aruco::Marker> MarkerSet;

  // int goal_detection_pass(cv::Mat& input, bool timing = false);
}

int rtx_goal_detection_main(int argc, char **argv);
