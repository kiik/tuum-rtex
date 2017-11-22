
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

  tuum::Navigator *gNav = nullptr;

  cv::Mat frameBuffer, frameBackBuffer;

  void setup() {
    rtx::object_detection_init();
    rtx::marker_detection_init();

    gNav = new tuum::Navigator();
    gSys.insmod(gNav);

    gGameField = new GameField();

    Basketball::setup();
  }

  void process() {
    hal::hw.readFrame(frameBackBuffer);
    frameBackBuffer.copyTo(frameBuffer);

    rtx::marker_detection(frameBuffer, gGameField);
    rtx::object_detection(frameBackBuffer, gGameField);

    gGameField->tick();

    Basketball::process();

    auto ctx = gNav->getContext();

    rtx::cmv_ui_set_nav(ctx);
    rtx::cmv_ui(frameBuffer, gGameField);
  }

}
