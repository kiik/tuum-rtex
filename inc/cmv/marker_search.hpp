
#include <vector>

#include <aruco/aruco.h>

namespace rtx {

  class GameField;

  typedef std::vector<aruco::Marker> MarkerSet;

  int marker_detection_init();
  int marker_detection(cv::Mat&, GameField*);

  void marker_detection_debug(cv::Mat&, bool render_3d);

}

int rtx_marker_detection_main(int argc, char **argv);
