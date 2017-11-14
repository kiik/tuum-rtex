
#include "robotex.hpp"

#include "tuum_platform.hpp"
#include "tuum_context.hpp"
#include "tuum_comm.hpp"

#include "core/rtx_RobotexCommSrv.hpp"

#include "rtx_goal_detect.hpp"

namespace rtx {

  tuum::System gSys;
  RobotexCommSrv gCommSrv;

  int ui_init(int argc, char* argv[]) {
    if(gCommSrv.init() < 0) return -1;
    if(tuum::wsocs::register_server(&gCommSrv) < 0) return -2;

    return 0;
  }

}

int main(int argc, char* argv[]) {
  // return rtx_goal_detection_main(argc, argv);

  tuum::setGlobalSystem(&(rtx::gSys));

  if(tuum::init(argc, argv) < 0) return -1;

  if(rtx::ui_init(argc, argv) < 0) {
    RTXLOG("UI initialization failed!", LOG_ERR);
    return -2;
  }

  rtx::gSys.setup();
  rtx::setup();

  while(1) {
    rtx::gSys.process();
    rtx::process();
  }

  /*
  if(usr::gTuumProgram != nullptr)
  return usr::gTuumProgram->run();
  */

  return 0;
}
