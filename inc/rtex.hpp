#ifndef RTEX_MODULE_H
#define RTEX_MODULE_H

#include "tuum_system.hpp"
#include "RobotexCommSrv.hpp"

int main(int, char*[]);

namespace rtex {

  extern tuum::System gSys;
  extern tuum::gui::RobotexCommSrv uiSrv;

  void setup();
  void process();

}

#endif // RTEX_MODULE_H
