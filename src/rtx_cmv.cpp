
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

  const cv::Scalar lightGray(211, 211, 211);
  const cv::Scalar goldenRod(32, 165, 218);
  const cv::Scalar deepPink(255, 20, 147);
  const cv::Scalar skyBlue(255, 191, 0);
  const cv::Scalar cWhite(255, 255, 255);
  const cv::Scalar cBlack(0, 0, 0);
  const cv::Scalar cRed(0, 0, 255);
  const cv::Scalar cGreen(0, 255, 0);

  struct thr_bounds_t {
    uint8_t low[3] = {255, 255, 255};
    uint8_t high[3] = {0, 0, 0};
  } gThr;

  struct input_t {
    int _ev, _key = 0;
    cv::Point mousePosition;
  } gInput;

  debug_flags_t gDebug;

  tuum::Navigator::ctx_t gNavCtx;

  const char *winTitle = "rtx::cmv_ui";
  int waitDelay = 1;

  bool cmv_ui_init = false;

  const int robotPlaneOffset = 100, axisPadding = 20, unitStep = 100, unitBarSize_2 = 10;

  cv::Mat oFrame;

  void cmv_ui_colorpick(int x, int y)
  {
    Vec3b pix = oFrame.at<Vec3b>(y, x);
    printf("COLORPICK - Lab(%i, %i, %i)\n", pix[0], pix[1], pix[2]);

    for(int i = 0; i < 3; ++i)
    {
      gThr.low[i] = min(gThr.low[i], pix[i]);
      gThr.high[i] = max(gThr.high[i], pix[i]);
    }
  }

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
        printf("M_LBTN_UP - pos=(%i, %i)\n", x, y);
        cmv_ui_colorpick(x, y);
        break;
      }
    }
  }

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
    if(!cmv_ui_init)
    {
      cv::namedWindow(winTitle, 1);
      cvSetMouseCallback(winTitle, cmv_mouse_handler, &gInput._ev);

      cmv_ui_init = true;
    }

    if(gDebug.axis_dbg_flag)
      cmv_ui_draw_axis(iFrame);

    if(gDebug.nav_dbg_flag)
    {
      {
        cv::Size sz = iFrame.size();
        cv::Point P0(sz.width / 2, sz.height - robotPlaneOffset);

        int H = 800;
        int W_2 = 1280 / 2, P0_y = 100;
        int P0_x = W_2;

        if(gNavCtx.hasTarget())
        {
          cv::Point tPos_world(gNavCtx.tPos.x, gNavCtx.tPos.y);
          cv::Point tPos(W_2 - tPos_world.y, H - tPos_world.x);

          cv::Point aPos_world, aPos;

          cv::Point mvec, oPos;

          cv::line(iFrame, P0, tPos, goldenRod, 2.5);
          cv::putText(iFrame, "T", tPos, FONT_HERSHEY_SIMPLEX, 0.4, cWhite, 1.4);

          float o = 0.0;

          // Get orientation target
          if(gNavCtx.hasOrient()) {
            o = gNavCtx.tOri;
          }
          else if(gNavCtx.hasAim()) {
            cv::Point aPos_world(gNavCtx.aPos.x, gNavCtx.aPos.y);
            cv::Point aPos(W_2 - aPos_world.y, H - aPos_world.x);

            cv::Point dv = (aPos - tPos);
            o = atan2(dv.y, dv.x);
          }

          if(o != 0.0)
          {
            const int L = 50;

            const float A = M_PI;
            mvec = cv::Point(cos(o) * L, sin(o) * L);

            cv::line(iFrame, tPos, tPos + mvec, cRed, 1.0);
            cv::putText(iFrame, "O", tPos + mvec, FONT_HERSHEY_SIMPLEX, 0.4, cWhite, 1.4);

            mvec = cv::Point(cos(o - 1.57) * L, sin(o - 1.57) * L);
            cv::line(iFrame, tPos, tPos + mvec, cGreen, 1.0);
          }

          // printf("(0)tPos World(%i, %i)/Frame(%i, %i)\n", tPos_world.x, tPos_world.y, tPos.x, tPos.y);
          // printf("(0)aPos World(%i, %i)/Frame(%i, %i)\n", aPos_world.x, aPos_world.y, aPos.x, aPos.y);
        }

        if(gNavCtx.hasAim())
        {
          cv::Point aPos_world(gNavCtx.aPos.x, gNavCtx.aPos.y);
          cv::Point aPos(W_2 - aPos_world.y, H - aPos_world.x);

          printf("(1)aPos World(%i, %i)/Frame(%i, %i)\n", aPos_world.x, aPos_world.y, aPos.x, aPos.y);

          cv::line(iFrame, P0, aPos, deepPink, 2.5);
          cv::putText(iFrame, "A", aPos, FONT_HERSHEY_SIMPLEX, 0.4, cWhite, 1.4);
        }

        int r = 55 / 2;

        cv::circle(iFrame, gInput.mousePosition, r, lightGray);
        cv::putText(iFrame, "X", gInput.mousePosition - cv::Point(5, -5), FONT_HERSHEY_SIMPLEX, 0.4, Scalar(0, 0, 255), 2.0);
      }

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
    }

    if(gDebug.marker_dbg_flag > 0)
    {
      bool r3d = gDebug.marker_dbg_flag >= 2 ? true : false;
      rtx::marker_detection_debug(iFrame, r3d);
    }

    if(gDebug.thr_en_flag)
    {
      cv::cvtColor(iFrame, oFrame, CV_BGR2Lab);
    }
    else oFrame = misc::resize(iFrame, 1280);
    cv::imshow(winTitle, oFrame);

    unsigned char c = cv::waitKey(waitDelay);

    if(isascii(c))
    {
      gInput._key = c;

      if(gInput._key == '0')
      {
        printf("PAUSE/RESUME %i\n", waitDelay);
        waitDelay = (waitDelay > 0 ? 0 : 1);
      }
      else if(gInput._key == '1')
      {
        gDebug.rect_en_flag = !gDebug.rect_en_flag;
      }
      else if(gInput._key == '2')
      {
        gDebug.marker_dbg_flag = (gDebug.marker_dbg_flag + 1) % 3;
      }
      else if(gInput._key == '3')
      {
        gDebug.nav_dbg_flag = !gDebug.nav_dbg_flag;
      }
      else if(gInput._key == '4')
      {
        gDebug.axis_dbg_flag = !gDebug.axis_dbg_flag;
      }
      else if(gInput._key == 'f')
      {
        const char* fp = "live-frame-dbg.png";
        printf("Saving current frame to '%s'.\n", fp);
        cv::imwrite(fp, iFrame);
      }
      else if(gInput._key == '5')
      {
        gDebug.stat_en_flag = !gDebug.stat_en_flag;
      }
      else if(gInput._key == '6')
      {
        if(!gDebug.thr_en_flag)
        {
          for(int i = 0; i < 3; ++i)
          {
            gThr.low[i] = 255;
            gThr.high[i] = 0;
          }
        }
        else
        {
          printf("THRESHOLD RANGE: ( ");
          for(int i = 0; i < 3; ++i)
          {
            printf("%i:%i ", gThr.low[i], gThr.high[i]);
          }
          printf(")\n");
        }

        gDebug.thr_en_flag = !gDebug.thr_en_flag;
      }
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

    // cv::imwrite("out.png", input);
    // img = cv::imread(argv[1]);

    return 0;
  }

}
