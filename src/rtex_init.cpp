
#include "rtex.hpp"

#include "tuum_platform.hpp"
#include "tuum_context.hpp"
#include "tuum_comm.hpp"

#include "RobotexCommSrv.hpp"

namespace rtex {

  tuum::System gSys;
  tuum::gui::RobotexCommSrv uiSrv;


  int ui_init(int argc, char* argv[]) {
    //tuum::gui::startup(argc, argv);

    if(uiSrv.init() < 0) return -1;
    if(tuum::wsocs::register_server(&uiSrv) < 0) return -2;

    return 0;
  }

}

int main(int argc, char* argv[]) {
  tuum::setGlobalSystem(&(rtex::gSys));

  if(tuum::init(argc, argv) < 0) return -1;

  if(rtex::ui_init(argc, argv) < 0) {
    RTXLOG("UI initialization failed!", LOG_ERR);
    return -2;
  }

  rtex::gSys.setup();
  rtex::setup();

  while(1) {
    rtex::gSys.process();
    rtex::process();
  }

  /*
  if(usr::gTuumProgram != nullptr)
  return usr::gTuumProgram->run();
  */

  return 0;
}
