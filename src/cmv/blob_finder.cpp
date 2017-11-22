

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *     Andrew Martignoni III
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Uses CMVision to retrieve the blob data
 */
// author Andy Martignoni III, Brian Gerkey, Brendan Burns, Ben Grocholsky, Brad Kratochvil

#include <time.h>

#include <boost/bind.hpp>

#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>

#include "platform.hpp"

#include "cmv/blob_finder.hpp"

using namespace tuum;

namespace cmv {

  CMVisionColorBlobFinder::CMVisionColorBlobFinder():
    debug_on_(false), width_(0), height_(0), color_filename_(""), blob_count_(0), vision_(NULL), mean_shift_on_(false), spatial_radius_(0),
    color_radius_(0), thr_select_({{255, 255, 255}, {0, 0, 0}})
  {

  }

  CMVisionColorBlobFinder::~CMVisionColorBlobFinder()
  {
    if (vision_)
    {
      delete vision_;
    }
  }

  bool CMVisionColorBlobFinder::initialize()
  {

    width_ = 0;
    height_ = 0;

    vision_ = new CMVision();

    blob_count_ = 0;

    color_filename_ = "./rtx_basketball.cmv";

    if (debug_on_)
    {
      cvNamedWindow("frame_bgr_buffer_");
      cvSetMouseCallback("frame_bgr_buffer_", CMVisionColorBlobFinder::DebugInputHandler, this);
    }

    return true;
  }

  void CMVisionColorBlobFinder::debugInputHandler(int ev, int x, int y, int flags)
  {
    switch(ev) {
      case CV_EVENT_LBUTTONUP:
      {
        image_pixel vec = frame_lab_buffer_.at<image_pixel>(y * 1280 + x);
        printf("ev:(%i,%i) -> pixel({.y = %i, .u = %i, .v = %i})\n", x, y, vec.l, vec.a, vec.b);

        uint8_t val;
        for(int c_ix = 0; c_ix < 3; ++c_ix)
        {
          val = *(reinterpret_cast<uint8_t*>(&vec) + c_ix);

          thr_select_.lo[c_ix] = MIN(val, thr_select_.lo[c_ix]);
          thr_select_.hi[c_ix] = MAX(val, thr_select_.hi[c_ix]);
        }

        printf("range = ( %i:%i %i:%i %i:%i )\n", thr_select_.lo[0], thr_select_.hi[0], thr_select_.lo[1], thr_select_.hi[1], thr_select_.lo[2], thr_select_.hi[2]);

        break;
      }
    }

  }

  int CMVisionColorBlobFinder::findAllBlobs(cv::Mat& cvImageRef_inp)
  {
    blob_callback_t cb = [](cmv::blob_t* ptr) {

    };

    return findAllBlobs(cvImageRef_inp, cb);
  }

  int CMVisionColorBlobFinder::findAllBlobs(cv::Mat& cvImageRef_inp, blob_callback_t blob_cb)
  {
    cv::Size size = cvImageRef_inp.size();
    cv::Mat debug_img_;

    time_ms_t t0 = millis();

    if ((size.width != width_) || (size.height != height_))
    {
      if (!(vision_->initialize(size.width, size.height)))
      {
        width_ = height_ = 0;
        return -1;
      }

      if (!color_filename_.empty())
      {
        if (!vision_->loadOptions(color_filename_.c_str())) return -2;
      }
      else return -3;

      width_ = size.width;
      height_ = size.height;
    }

    //if (mean_shift_on_)
      // cvPyrMeanShiftFiltering(cvImage, cvImage, spatial_radius_, color_radius_);

    cvImageRef_inp.copyTo(frame_bgr_buffer_);
    cv::cvtColor(cvImageRef_inp, cvImageRef_inp, CV_BGR2Lab);
    cvImageRef_inp.copyTo(frame_lab_buffer_);

    if(debug_on_)
    {
      //debug_img_ = cvCloneImage(cvImage);
      //frame_bgr_buffer_.copyTo(debug_img_);
    }

    // Find the color blobs
    if (!vision_->processFrame(reinterpret_cast<image_pixel*> (frame_lab_buffer_.data))) return -4;

    // Get all the blobs
    blob_count_ = 0;
    cmv::blob_t blob_buf;
    for (int ch = 0; ch < CMV_MAX_COLORS; ++ch)
    {
      // Get the descriptive color
      rgb c = vision_->getColorVisual(ch);
      char* name = vision_->getColorName(ch);

      // Grab the regions for this color
      CMVision::region* r = NULL;

      for (r = vision_->getRegions(ch); r != NULL; r = r->next)
      {
        if (debug_on_)
        {
          cv::rectangle(frame_bgr_buffer_, cvPoint(r->x1, r->y1), cvPoint(r->x2, r->y2), CV_RGB(c.red, c.green, c.blue));
        }

        // Basic info
        blob_buf.name = name;
        blob_buf.c = {c.red, c.green, c.blue};

        // Stats
        blob_buf.area = r->area;

        blob_buf.c_x = r->cen_x; // rint(r->cen_x + .5);
        blob_buf.c_y = r->cen_y; // rint(r->cen_y + .5);

        // Rect
        blob_buf.left = r->x1;
        blob_buf.right = r->x2;
        blob_buf.top = r->y1;
        blob_buf.bottom = r->y2;

        blob_cb(&blob_buf);

        blob_count_++;
      }
    }

    passData.blobCount = blob_count_;

    if (debug_on_)
    {
      time_ms_t t1 = millis();
      printf("dt=%lu, %.1fHz\n", (t1 - t0), (1000.0 / (t1 - t0)));

      // cvCvtColor(debug_img_, debug_img_, CV_RGB2YUV);
      cv::imshow("frame_bgr_buffer_", frame_bgr_buffer_);

      // cvShowImage("Lab Image", debug_img_);

      cvWaitKey(3);
    }
    return 0;
  }

}
