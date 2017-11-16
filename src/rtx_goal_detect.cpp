
#include <ctime>
#include <iostream>

#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "hal.hpp"

// #include "cmv/cmvision.h"

#include "tuum_visioning.hpp"

#include "core/rtx_GameField.hpp"

#include "robotex.hpp"

#include "rtx_goal_detect.hpp"

using namespace cv;
using namespace std;
using namespace aruco;

/*
class CMVProcessor
{
public:
  CMVision cmvision; // Instance of the cmvision class

  CMVProcessor(const PixelClassifierSettings& settings);

  void init()
  {

  }

  void processFrame(cv::Mat& in); // Invoke cmvision.processFrame()
  {
    //uyvy* frame
    using namespace CMVision;

    cv::Size sz = in.size();
    image img = {in.dat, sz.width, sz.height};

    ThresholdImage(cmap, img, tmap);
  }

};*/

namespace misc {
  cv::Mat resize(const cv::Mat &in, int width) {
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

  enum FrameType {
    REGULAR,
    PS4EYE,
  } frameType = PS4EYE;

  int init()
  {
    return 0;

    if(vs == nullptr) vs = new VideoCapture(1);
    if(!vs->isOpened()) {
      cout << "cannot open camera" << endl;
      return -1;
    }

    if(frameType != PS4EYE)
    {
      vs->set(CV_CAP_PROP_FRAME_WIDTH,1280);
      vs->set(CV_CAP_PROP_FRAME_HEIGHT,800);
    }

    vs->grab();

    cv::Mat buf;
    vs->retrieve(buf);
    cv::Size sz = buf.size();

    cout << sz << endl;

    return 0;
  }

  int process()
  {
    if(vs != nullptr) return vs->grab();
    return 0;
  }

  int read_frame(cv::Mat& output)
  {
    return tuum::hal::hw.readFrame(output);

    vs->retrieve(output);

    if(frameType == PS4EYE)
    {
      cv::Rect gArea(48, 0, 1280, 800);
      output = output(gArea);
    }

    return 0;
  }

}}

namespace rtx {

  MarkerDetector gDetector;
  MarkerSet gMarkers;
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

  int ball_detection_init();

  int init()
  {
    ball_detection_init();

    gGameField = new GameField();

    return 0;
  }

  int load_camera_transform(cv::Size size)
  {
    const char* fp = "/cam.yml";
    const char* ps4_fp = "./ps4eye.yml";

    if(rtx::hal::frameType == rtx::hal::PS4EYE)
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

namespace rtx {

  cv::SimpleBlobDetector gBallDetect;

  std::vector<KeyPoint> keypoints, field_kps;
  cv::Mat ball_img_buf, ball_mask, field_mask;

  tuum::Visioning gVision;

  Vec3b pickedMax(0, 0, 0), pickedMin(255, 255, 255);

  bool colorpick = true;
  int pickX, pickY;

  struct color_range_t {
    cv::Scalar lower, upper;
  };

  int ball_detection_init()
  {
    // gVision.init();
    return 0;
  }

  int ball_detection_pass(cv::Mat& in)
  {
    //ball_img_buf = in;
    //gVision.doPass(in, ball_img_buf);

    //cv::Scalar(35, 40, 40), cv::Scalar(60, 255, 150)

    /* Floor
    cv::Scalar lower(0, 20, 150);
    cv::Scalar upper(15, 40, 200);
    */

    cv::cvtColor(in, ball_img_buf, CV_BGR2HSV);

    /* RGB Ball
    cv::Scalar lower(20, 30, 20);
    cv::Scalar upper(100, 250, 120);
    */

    cv::Scalar lower(19, 117, 50);
    cv::Scalar upper(85, 250, 190);

    color_range_t goal0_hsv = {
      {120, 117, 61},
      {180, 241, 136}
    };

    color_range_t floor_hsv = {
      {1, 209, 60},
      {180, 255, 200}
    };

    cv::inRange(ball_img_buf, floor_hsv.lower, floor_hsv.upper, field_mask);
    //bitwise_not(field_mask, field_mask);

    //cv::inRange(ball_img_buf, goal0_hsv.lower, goal0_hsv.upper, ball_mask);

    gBallDetect.detect(field_mask, field_kps);

    //res = cv2.bitwise_and(frame,frame, mask= mask)

    //ball_img_buf = in;
  }

  void ball_detection_debug(cv::Mat& out)
  {
    //ball_img_buf.copyTo(out, field_mask);
    out = field_mask;


    if(colorpick)
    {
      colorpick = false;

      cv::Vec3b hsv = ball_img_buf.at<Vec3b>(Point(pickX, pickY));

      for(int i = 0; i < 3; ++i)
      {
        pickedMin[i] = min(pickedMin[i], hsv[i]);
        pickedMax[i] = max(pickedMax[i], hsv[i]);
      }

      printf("image[%i,%i] = (%i, %i, %i). Range {%i, %i, %i} - {%i, %i, %i}\n", pickX, pickY, hsv[0], hsv[1], hsv[2], pickedMin[0], pickedMin[1], pickedMin[2], pickedMax[0], pickedMax[1], pickedMax[2]);
    }

    cv::drawKeypoints(field_mask, field_kps, out, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
  }

  void colorpick_debug(int x, int y)
  {
    pickX = x;
    pickY = y;
    colorpick = true;
  }

  void colorpick_clear()
  {
    pickedMax = Vec3b(0, 0, 0);
    pickedMin = Vec3b(255, 255, 255);
  }

}

void rtx_cv_mouseHandler(int event, int x, int y, int flags, void* param)
{
  switch(event)
  {
    case CV_EVENT_LBUTTONDOWN:
      if(flags & CV_EVENT_FLAG_CTRLKEY)
        printf("Left button down with CTRL pressed\n");
      break;
    case CV_EVENT_LBUTTONUP:
    {
      rtx::colorpick_debug(x, y);
      break;
    }
  }
}

int rtx_goal_detection_main(int argc,char **argv)
{
  using namespace rtx;

  if(rtx::hal::init() < 0) return -1;
  if(rtx::init() < 0) return -2;

  {
    cv::Mat frame;
    //*rtx::hal::vs >> frame;
    tuum::hal::hw.readFrame(frame);
    rtx::load_camera_transform(frame.size());
  }

  cv::Mat img, img_buf;
  tuum::hal::hw.readFrame(img);

  int res, waitTime = 0;
  char key = 0;

  const char* TITLE_DBG = "Tuum Devel - Debug";

  int mouseParam = CV_EVENT_FLAG_LBUTTON;

  cv::namedWindow(TITLE_DBG, 1);
  cvSetMouseCallback(TITLE_DBG, rtx_cv_mouseHandler, &mouseParam);

  do
  {
    tuum::hal::process();
    gSys.process();

    tuum::hal::hw.readFrame(img);
    res = rtx::goal_detection_pass(img);

    cv::Size sz = img.size();
    rtx::gGameField->digestGoalMarkers(rtx::gMarkers, sz.width / 2.0, sz.height);

    rtx::ball_detection_pass(img);

    img.copyTo(img_buf);
    rtx::goal_detection_debug(img_buf);
    rtx::ball_detection_debug(img_buf);

    //cv::namedWindow("Tuum Devel - Input", 1);
    //cv::imshow("Tuum Devel - Input", misc::resize(img, 1280));

    cv::imshow(TITLE_DBG, img_buf); // misc::resize(, 1280);

    key = cv::waitKey(waitTime); // wait for key to be pressed
    if(key == 's') {
      waitTime = waitTime == 0 ? 10 : 0;
    } else if(key == 'c') {
      rtx::colorpick_clear();
    }

  } while(key != 27);

  cv::waitKey(0);

  // cv::imwrite("out.png", input);
  // img = cv::imread(argv[1]);

  return 0;
}
