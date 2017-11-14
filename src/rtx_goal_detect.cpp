
#include <ctime>
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rtx_goal_detect.hpp"

using namespace cv;
using namespace std;
using namespace aruco;

namespace misc {
  cv::Mat resize(const cv::Mat &in, int width){
    if (in.size().width<=width) return in;
    float yf=float(  width)/float(in.size().width);
    cv::Mat im2;
    cv::resize(in,im2,cv::Size(width,float(in.size().height)*yf));
    return im2;
  }
}

namespace rtx {
namespace hal {
  VideoCapture *vs = nullptr;

  int init()
  {
    if(vs == nullptr) vs = new VideoCapture(0);
    if(!vs->isOpened()) {
      cout << "cannot open camera" << endl;
      return -1;
    }

    vs->grab();

    return 0;
  }

  int process()
  {
    return vs->grab();
  }

  int read_frame(cv::Mat& output)
  {
    vs->retrieve(output);
    return 0;
  }

}}

namespace rtx {

  MarkerDetector gDetector;
  vector<Marker> gMarkers;
  std::map<uint32_t, MarkerPoseTracker> gTrackers;

  float gMarkerSize = 0.1475; // 147.5mm
  CameraParameters gCamParams;
  bool cam_params_avail = false;

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

  lap_timer_t goal_detection_tmr;

  int init()
  {

    return 0;
  }

  int load_camera_transform(const char* fp, cv::Size size)
  {
    gCamParams.readFromXMLFile(fp);

    if(gCamParams.isValid()) {
      gCamParams.resize(size);
      cam_params_avail = true;
    }

    return 0;
  }

  int goal_detection_pass(cv::Mat& input, bool timing = false)
  {
    cv::Size sz = input.size(); // cv::Size ~ {.width, .height}

    if(timing) goal_detection_tmr.start();
    gDetector.detect(input, gMarkers);
    if(timing) {
      goal_detection_tmr.time();
      goal_detection_tmr.print();
    }

    for(auto &marker : gMarkers)
    {
      gTrackers[marker.id].estimatePose(marker, gCamParams, gMarkerSize);
    }

    return 0;
  }

  void goal_detection_debug(cv::Mat& input)
  {
    bool render_3d = gCamParams.isValid() && gMarkerSize != -1;

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

int rtx_goal_detection_main(int argc,char **argv)
{
  using namespace rtx;

  if(hal::init() < 0) return -1;
  if(rtx::init() < 0) return -2;

  {
    cv::Mat frame;
    *hal::vs >> frame;
    rtx::load_camera_transform("./cam.yml", frame.size());
  }

  cv::Mat img, img_buf;
  hal::read_frame(img);

  int res, waitTime = 0;
  char key = 0;

  do
  {
    hal::read_frame(img);
    res = rtx::goal_detection_pass(img);

    img.copyTo(img_buf);
    rtx::goal_detection_debug(img_buf);

    cv::namedWindow("Tuum Devel", 1);
    cv::imshow("Tuum Devel", misc::resize(img_buf, 1280));

    key = cv::waitKey(waitTime); // wait for key to be pressed
    if(key == 's') {
      waitTime = waitTime == 0 ? 10 : 0;
    }

  } while(key != 27 && hal::process());

  cv::waitKey(0);

  // cv::imwrite("out.png", input);
  // img = cv::imread(argv[1]);

  return 0;
}
