
#include "rtex.hpp"
#include "tuum_context.hpp"

#include "STM.hpp"
#include "rtx_ctl.hpp"

#include "tuum_motion.hpp"
#include "tuum_context.hpp"

using namespace tuum;

namespace rtex {

  STM* stm;

  void setup() {
    tuum::gMotion->setAimTarget({10, 10});
  }

  void process() {

  }

}
