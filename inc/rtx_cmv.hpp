
#ifndef RTX_CMV_H
#define RTX_CMV_H

#include "cmv/object_search.hpp"
#include "cmv/marker_search.hpp"

#include "tuum_navigator.hpp"

namespace rtx {

  struct debug_flags_t {
    bool rect_en_flag = false, nav_dbg_flag = true, axis_dbg_flag = true;
    bool stat_en_flag = true, thr_en_flag = false;
    uint8_t marker_dbg_flag = 0;
  };

  extern debug_flags_t gDebug;

  class GameField;

  extern cv::Mat gCamMx, gDistCoeff;

  void cmv_ui_set_nav(tuum::Navigator::ctx_t);

  void cmv_ui(cv::Mat&, GameField*);

  int cmv_tests_main(int argc, char* argv[]);

}

#endif
