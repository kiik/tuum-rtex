#ifndef ROBOTEX_H
#define ROBOTEX_H

#include "tuum_system.hpp"

int main(int, char*[]);

namespace rtx {

  class RobotexCommSrv;
  class GameField;

  extern tuum::System gSys;

  extern RobotexCommSrv gCommSrv;
  extern GameField* gGameField;

  int init();
  void setup();
  void process();

}

#endif
