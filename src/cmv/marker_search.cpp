
#include <ctime>
#include <iostream>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include "hal.hpp"

#include "tuum_visioning.hpp"

#include "core/rtx_GameField.hpp"

#include "robotex.hpp"

#include "cmv/marker_search.hpp"

using namespace cv;
using namespace std;
using namespace aruco;

namespace rtx {

  struct lap_timer_t {
    clock_t t0, t1;
    double m_dt;

    void start() { t0 = clock(); }
    void stop() { t1 = clock(); }

    double time()
    {
      stop();
      m_dt = double(t1 - t0) / CLOCKS_PER_SEC * 1000;
    }

    void print()
    {
      cout << m_dt << "ms" << endl;
    }

  };

}

namespace rtx {

  MarkerDetector gDetector;
  MarkerSet gMarkers;
  std::map<uint32_t, MarkerPoseTracker> gTrackers;

  float gMarkerSize = 0.1475; // 147.5mm
  CameraParameters gCamParams;
  bool cam_params_avail = false;

  lap_timer_t marker_detection_tmr;

  enum FrameType {
    DEFAULT,
    PS4EYE,
  } frameType = PS4EYE;

  int load_camera_transform(cv::Size size)
  {
    const char* fp = "/cam.yml";
    const char* ps4_fp = "./ps4eye.yml";

    if(frameType == PS4EYE)
    {
      gCamParams.readFromXMLFile(ps4_fp);
    }
    else gCamParams.readFromXMLFile(fp);

    if(gCamParams.isValid()) {
      gCamParams.resize(size);
      cam_params_avail = true;
    }

    return 0;
  }

  int marker_detection_init()
  {
    return 0;
  }

  int marker_detection(cv::Mat& input, GameField *gmField)
  {
    bool timing = false;

    cv::Size sz = input.size(); // cv::Size ~ {.width, .height}

    if(!cam_params_avail) load_camera_transform(sz);

    if(timing) marker_detection_tmr.start();

    gDetector.detect(input, gMarkers);

    if(timing) {
      marker_detection_tmr.time();
      marker_detection_tmr.print();
    }

    for(auto &marker : gMarkers)
    {
      gTrackers[marker.id].estimatePose(marker, gCamParams, gMarkerSize);
      gmField->digestGoalMarker(marker);
    }

    return 0;
  }

  void marker_detection_debug(cv::Mat& input, bool render_3d)
  {
    if(!gCamParams.isValid() || gMarkerSize == -1) render_3d = false;

    Marker *ptr;
    for(unsigned int i=0;i < gMarkers.size(); i++) {
      ptr = &gMarkers[i];
      ptr->draw(input, Scalar(0,0,255), 1);

      if(render_3d)
      {
        CvDrawingUtils::draw3dCube(input, *ptr, gCamParams);
        CvDrawingUtils::draw3dAxis(input, *ptr, gCamParams);
      }

      // cout << *ptr << endl;
    }
  }

}
