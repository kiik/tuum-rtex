
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "tuum_visioning.hpp"
#include "tuum_localization.hpp"
#include "tuum_navigator.hpp"
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "STM.hpp"

#include "hal.hpp"

#include "core/rtx_LogicManager.hpp"
#include "core/rtx_GameField.hpp"
#include "game/rtx_basketball.hpp"

#include "rtx_cmv.hpp"
#include "rtx_ctl.hpp"

#include "robotex.hpp"

#include "MainBoard.hpp"

using namespace tuum;

namespace rtx {

  GameField *gGameField = nullptr;

  cv::Mat frameBuffer;

  void setup() {
    rtx::object_detection_init();
    rtx::marker_detection_init();

    gSys.insmod(new tuum::Navigator());

    gGameField = new GameField();

    Basketball::setup();
  }

  void process() {
    hal::hw.readFrame(frameBuffer);

    rtx::object_detection(frameBuffer, gGameField);
    rtx::marker_detection(frameBuffer, gGameField);

    Basketball::process();
  }

}
