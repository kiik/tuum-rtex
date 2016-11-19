
#include "tuum_logger.hpp"

#include "hal.hpp"

#include "rtx_cmds.hpp"

namespace tuum { namespace cmds {

  int drive(float spd, float dir, float rot) {
    hal::hw.getMotorControl()->omniDrive(spd, dir, rot);
  }

  int dribble(float pwr) {
    hal::hw.getMainBoard()->startDribbler(pwr);
  }

  int kickcoil() {
    hal::hw.getMainBoard()->coilKick();
  }

}}
