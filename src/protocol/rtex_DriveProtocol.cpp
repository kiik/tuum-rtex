
#include "rtx_cmds.hpp"

#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "protocol/rtex_DriveProtocol.hpp"

using namespace tuum::wsocs;

namespace tuum {

  DriveProtocol::DriveProtocol() {

  }

  WSProtocol::route_t DriveProtocol::getDescriptor()
  {
    WSProtocol::route_t out;
    out.uri = "/drv";
    out.wsp = this;
    return out;
  }

  int DriveProtocol::route(const WSProtocol::Message& m) {
    std::string cmd = m.dat[WSProtocol::JS_CMD];

    if(cmd == "drv") {
      return drive(m.dat);
    } else if(cmd == "info") {
      return getInfo(m.dat);
    }

    return -1;
  }

  //TODO: Error catching
  int DriveProtocol::drive(json dat) {
    tuum::cmds::drive(dat["s"], dat["d"].get<float>() / 1000.0, dat["r"]);
  }

  int DriveProtocol::getInfo(const json& dat) {
    json out;
    tuum::gMotion->toJSON(out);
    send(out);
  }

}
