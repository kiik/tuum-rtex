
#include "rtex.hpp"

#include "tuum_context.hpp"

#include "STM.hpp"
#include "rtx_ctl.hpp"
#include "LogicManager.hpp"

using namespace tuum;

namespace rtex {

  STM* stm;

  Timer debugTmr;

  void setup() {
    stm = LogicManager::loadOffensivePlay();//new STM();

    stm->setup();
    debugTmr.start(2000);
  }

  void process() {
    //stm->process();

    if(debugTmr.isTime()) {
      //gMotion->debug();
      debugTmr.start(2000);
    }
  }

}
