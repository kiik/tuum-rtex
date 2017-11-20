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

#ifndef RTX_BLOB_FINDER_H
#define RTX_BLOB_FINDER_H

#include <opencv2/opencv.hpp>

#include "cmv/conv.h"
#include "cmv/cmvision.h"


#define CMV_NUM_CHANNELS CMV_MAX_COLORS
#define CMV_HEADER_SIZE 4*CMV_NUM_CHANNELS
#define CMV_BLOB_SIZE 16
#define CMV_MAX_BLOBS_PER_CHANNEL 10

#define DEFAULT_CMV_WIDTH CMV_DEFAULT_WIDTH
#define DEFAULT_CMV_HEIGHT CMV_DEFAULT_HEIGHT

namespace cmv {

  struct blob_t {
    std::string name;
    int x0, x1, y0, y1;

    int area;
    int c_x, c_y;
    int left, right, top, bottom;

    struct {
      uint8_t r, g, b;
    } c;
  };

  typedef void(*blob_callback_t)(cmv::blob_t*);

  class CMVisionColorBlobFinder
  {
  public:

    struct pass_t {
      unsigned int blobCount;
    } passData;

    /// \brief Constructor
    CMVisionColorBlobFinder();

    /// \brief Destructor
    virtual ~CMVisionColorBlobFinder();

    /*! \brief initialization function
     * @param node_handle
     * @return true if initialization successful, false else.
     */
    bool initialize();

    /// \brief Image callback
    int findAllBlobs(cv::Mat&);
    int findAllBlobs(cv::Mat&, blob_callback_t);

    // cv::Mat* getDebugFramePtr() { return &debug_frame_; }

    static void DebugInputHandler(int event, int x, int y, int flags, void* param)
    {
      ((CMVisionColorBlobFinder*)param)->debugInputHandler(event, x, y, flags);
    }

    void debugInputHandler(int, int, int, int);

  private:

    struct thr_select_t {
      int lo[3], hi[3];
    } thr_select_;

    // ros::Publisher blob_publisher_;
    // ros::Subscriber image_subscriber_;

    // sensor_msgs::Image image;

    cv::Mat frame_bgr_buffer_, frame_lab_buffer_;

    bool debug_on_;
    uint16_t width_;
    uint16_t height_;
    std::string color_filename_;

    unsigned int blob_count_;
    int input_event_;

    CMVision *vision_;

    // cmvision::Blobs blob_message_;

    bool mean_shift_on_;
    double spatial_radius_;
    double color_radius_;

  };

}

#endif
