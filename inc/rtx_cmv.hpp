
#ifndef RTX_CMV_H
#define RTX_CMV_H

#include "cmv/object_search.hpp"
#include "cmv/marker_search.hpp"

#include "tuum_navigator.hpp"

namespace rtx {

  class GameField;

  void cmv_ui_set_nav(tuum::Navigator::ctx_t);

  void cmv_ui(cv::Mat&, GameField*);

  int cmv_tests_main(int argc, char* argv[]);

}

#endif
