
#include "rtex.hpp"
#include "tuum_context.hpp"

#include "STM.hpp"
#include "rtx_ctl.hpp"

#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "tuum_visioning.hpp"
#include "tuum_localization.hpp"
#include "tuum_navigation.hpp"
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "hal.hpp"


using namespace tuum;

namespace rtex {


  STM* stm;
  //test

  void setup() {

    Ball* b = nullptr;
    if(b != nullptr){
      b = gNavigation->getNearestBall();
      vec2i pos = gNavigation->calcBallPickupPos(b->getTransform()).getPosition();
      tuum::gMotion->setAimTarget(pos);
    }
  }

  void process() {

  }

}
