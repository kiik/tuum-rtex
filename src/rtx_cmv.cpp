
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
        // rtx::colorpick_debug(x, y);
        break;
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

      //cv::namedWindow("Tuum Devel - Input", 1);
      //cv::imshow("Tuum Devel - Input", misc::resize(img, 1280));

      cv::imshow(TITLE_DBG, img); // misc::resize(, 1280);

      key = cv::waitKey(waitTime); // wait for key to be pressed
      if(key == 's') {
        waitTime = waitTime == 0 ? 10 : 0;
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
