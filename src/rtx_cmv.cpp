
#include <stdlib.h>
#include <math.h>                           /* math functions */

#include <opencv/highgui.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "hal.hpp"

#include "robotex.hpp"

#include "cmv/object_search.hpp"
#include "cmv/marker_search.hpp"

#include "rtx_cmv.hpp"

#include "core/rtx_GameField.hpp"

#define PS4EYE_IMGL cv::Rect(48, 0, 1280, 800)

using namespace cv;
using namespace std;

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

  struct input_t {
    int _ev, _key;
    cv::Point mousePosition;
  } gInput;

  struct debug_flags_t {
    bool rect_en_flag = false;
  } gDebug;

  tuum::Navigator::ctx_t gNavCtx;

  int waitDelay = -1;

  bool cmv_ui_init = false;

  void cmv_mouse_handler(int event, int x, int y, int flags, void* param)
  {
    switch(event)
    {
      case CV_EVENT_MOUSEMOVE:
        gInput.mousePosition = cv::Vec2i(x, y);
        break;
      case CV_EVENT_LBUTTONDOWN:
        if(flags & CV_EVENT_FLAG_CTRLKEY)
          printf("Left button down with CTRL pressed\n");
        break;
      case CV_EVENT_LBUTTONUP:
      {
        // rtx::colorpick_debug(x, y);
        printf("M_LBTN_UP - pos=(%i, %i)\n", x, y);
        break;
      }
    }
  }

  const int robotPlaneOffset = 100, axisPadding = 20, unitStep = 100, unitBarSize_2 = 10;

  void cmv_ui_draw_axis(cv::Mat& iFrame)
  {
    cv::Size sz = iFrame.size();

    const int W_2 = sz.width / 2;
    int y0 = sz.height - robotPlaneOffset, dx, x1, x2, y1, y2;

    cv::Point p0 = cv::Point(axisPadding, y0);
    cv::Point p1 = cv::Point(sz.width - axisPadding, y0);
    cv::Scalar color = cv::Scalar(0, 0, 255);

    // Draw X axis
    cv::line(iFrame, p0, p1, color, 1);
    cv::putText(iFrame, "x", p1 + cv::Point(2, -2), FONT_HERSHEY_SIMPLEX, 0.6, color, 1.4);

    dx = abs(p1.x - p0.x);

    x1 = W_2 + unitStep;
    x2 = W_2 - unitStep;

    y1 = y0 + unitBarSize_2;
    y2 = y0 - unitBarSize_2;

    // Draw unit bars
    while(dx > 0)
    {
      cv::line(iFrame, {x1, y1}, {x1, y2}, color, 1.8);
      cv::line(iFrame, {x2, y1}, {x2, y2}, color, 1.8);

      x1 += unitStep;
      x2 -= unitStep;
      dx -= unitStep;
    }

    p0 = cv::Point(W_2, y0);
    p1 = cv::Point(W_2, axisPadding);
    color = cv::Scalar(0, 255, 0);

    // Draw Y Axis
    cv::line(iFrame, p0, p1, color, 1);
    cv::putText(iFrame, "y", p1 + cv::Point(2, -2), FONT_HERSHEY_SIMPLEX, 0.6, color, 1.4);

    x1 = W_2 - unitBarSize_2;
    x2 = W_2 + unitBarSize_2;

    y1 = p0.y;

    while(y1 > p1.y)
    {
      cv::line(iFrame, {x1, y1}, {x2, y1}, color, 1.8);

      y1 -= unitStep;
    }
  }

  void cmv_ui_set_nav(tuum::Navigator::ctx_t ctx)
  {
    gNavCtx = ctx;
  }

  void cmv_ui(cv::Mat& iFrame, GameField *gmField)
  {
    const char *winTitle = "rtx::cmv_ui";

    if(!cmv_ui_init)
    {
      cv::namedWindow(winTitle, 1);
      cvSetMouseCallback(winTitle, cmv_mouse_handler, &gInput._ev);

      cmv_ui_init = true;
    }

    cmv_ui_draw_axis(iFrame);

    const cv::Scalar lightGray(211, 211, 211);
    const cv::Scalar goldenRod(32, 165, 218);

    {
      cv::Size sz = iFrame.size();
      cv::Point P0(sz.width / 2, sz.height - robotPlaneOffset);

      const cv::Scalar neonGreen(0, 255, 57);

      BallSet *bls = gmField->getBallsHandle();

      if(bls != nullptr)
      {
        for(auto &bl : *bls)
        {
          tuum::Vec2i _pos = bl->getBlob()->getCentroid();
          cv::Vec2i pos = cv::Vec2i(_pos.x, _pos.y);
          auto rect = bl->getBlob()->getRect();

          const std::string label = bl->toString(); // tuum::format("B#%lu(%i,%i)", bl->getId(), pos[0], pos[1]);

          cv::line(iFrame, P0, pos, lightGray, 1.0);

          cv::putText(iFrame, label, pos,FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0,0,0,255),2.0);
          cv::putText(iFrame, label, pos,FONT_HERSHEY_SIMPLEX, 0.35, Scalar(255,255,255,255),1.7);

          if(gDebug.rect_en_flag)
          {
            cv::rectangle(iFrame, cv::Point(rect.x0, rect.y0), cv::Point(rect.x1, rect.y1), neonGreen);
          }
        }
      }
    }

    {
      cv::Size sz = iFrame.size();
      cv::Point P0(sz.width / 2, sz.height - robotPlaneOffset);

      const cv::Scalar skyBlue(255, 191, 0);

      GoalSet *gls = gmField->getGoalsHandle();

      if(gls != nullptr)
      {
        for(auto &gl : *gls)
        {
          tuum::Vec2i _pos = gl->getBlob()->getCentroid();
          cv::Vec2i pos = cv::Vec2i(_pos.x, gl->getBlob()->getRect().y1);
          auto rect = gl->getBlob()->getRect();

          const std::string label = gl->toString(); // tuum::format("G#%lu(%i,%i)", gl->getId(), pos[0], pos[1]);

          cv::line(iFrame, P0, pos, skyBlue, 1.0);

          cv::putText(iFrame, label, pos,FONT_HERSHEY_SIMPLEX, 0.35, Scalar(0,0,0,255),2.0);
          cv::putText(iFrame, label, pos,FONT_HERSHEY_SIMPLEX, 0.35, Scalar(255,255,255,255),1.7);

          if(gDebug.rect_en_flag)
          {
            cv::rectangle(iFrame, cv::Point(rect.x0, rect.y0), cv::Point(rect.x1, rect.y1), skyBlue);
          }
        }
      }
    }

    {
      cv::Size sz = iFrame.size();
      cv::Point P0(sz.width / 2, sz.height - robotPlaneOffset);

      int r = 55 / 2;

      // pos = (930, 289), diameter=55pix
      // 0,020846495 change per 1cm

      if(gNavCtx.hasTarget())
      {
        cv::Point tPos(gNavCtx.tPos.x, gNavCtx.tPos.y);
        cv::line(iFrame, P0, tPos, goldenRod, 2.0);
        cv::putText(iFrame, "T", tPos, FONT_HERSHEY_SIMPLEX, 0.5, goldenRod, 2.0);
      }

      cv::circle(iFrame, gInput.mousePosition, r, lightGray);
      cv::putText(iFrame, "X", gInput.mousePosition - cv::Point(5, -5), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255, 255), 2.0);
    }

    cv::imshow(winTitle, iFrame);

    gInput._key = cv::waitKey(waitDelay);

    if(gInput._key == 's')
    {
      waitDelay = (waitDelay >= 0 ? -1 : 1);
    }
    else if(gInput._key == 'r')
    {
      gDebug.rect_en_flag = !gDebug.rect_en_flag;
    }
  }

  int cmv_tests_main(int argc,char **argv)
  {
    using namespace rtx;

    cv::Mat img, imgBuf;
    int res, waitTime = 0;
    char key = 0;

    const char* TITLE_DBG = "Debug";

    int mouseParam = CV_EVENT_FLAG_LBUTTON;

    // cv::namedWindow(TITLE_DBG, 1);
    // cvSetMouseCallback(TITLE_DBG, rtx_cv_mouseHandler, &mouseParam);

    rtx::object_detection_init();
    rtx::marker_detection_init();

    do
    {
      tuum::hal::process();
      gSys.process();

      // Read video stream
      img = cv::imread("./basketball_demo_00.png");
      img = img(PS4EYE_IMGL);

      // Do image processing
      rtx::object_detection(img, gGameField);
      // rtx::marker_detection(img, gGameField);

      cv::imshow(TITLE_DBG, img); // misc::resize(, 1280);

      key = cv::waitKey(waitTime);

      if(key == 's') {
        waitTime = waitTime == -1 ? 1 : -1;
      } else if(key == 'c') {
        // rtx::colorpick_clear();
      } else if(key == 'b') {
        // rtx::balls_debug();
      }

    } while(key != 27);

    cv::waitKey(0);

    // cv::imwrite("out.png", input);
    // img = cv::imread(argv[1]);

    return 0;
  }

}
