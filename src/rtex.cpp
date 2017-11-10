
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "tuum_visioning.hpp"
#include "tuum_localization.hpp"
#include "tuum_navigator.hpp"
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "hal.hpp"

#include "rtex.hpp"

#include "STM.hpp"
#include "LogicManager.hpp"

#include "rtx_ctl.hpp"
#include "rtx_fb.hpp"

#include "MainBoard.hpp"

using namespace tuum;

namespace rtex {

  STM* stm;
  //test

  Timer debugTmr;

  void setup() {
    debugTmr.start(2000);

    //hal::hw.getMainBoard()->startDribbler(0.1);

    //stm = LogicManager::loadOffensivePlay();//new STM();

    //stm->setup();
    FBLogic::setup();
  }

  bool dbg = false;

  void process() {
    if(!dbg) {
      dbg = true;
      hal::hw.getMotorControl()->omniDrive(0, 0, 0);
    }

    //stm->process();
    FBLogic::process();

    if(debugTmr.isTime()) {
      //gMotion->debug();
      debugTmr.start(2000);
    }

  }

}
