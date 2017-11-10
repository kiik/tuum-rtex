
#include "rtx_cmds.hpp"

#include "tuum_motion.hpp"
#include "tuum_context.hpp"
#include "tuum_system.hpp"

#include "protocol/rtex_DriveProtocol.hpp"

using namespace tuum::wsocs;

namespace tuum {

  DriveProtocol::DriveProtocol():
    WSProtocol({
      "Drive Protocol",
      "/drv",
      "0.0.1-al.0",
      {
        {"Omni Drive", "/omni", {
            {"MotorPower", "mp", WSType::WST_Integer},
            {"Motor Valve", "mv", WSType::WST_Integer},
            {"Joint Valve", "jv", WSType::WST_Integer},
          }
        },
        {"Drive Signal", "/drvsig", {
            {"Signal Name", "s", WSType::WST_String},
          }
        },
        {"Get Drive Feedback", "/fb", {}},

        {"Get Control Mode", "/ctlm.get", {}},
        {"Set Control Mode", "/ctlm.set", {
          {"Control Mode", "ctlm", WSType::WST_String}
        }}
      },
      this
    })
  {

  }

  int DriveProtocol::route(const WSProtocol::Message& m) {
    std::string uri = m.dat[WSProtocol::JS_URI];

    if(uri == "/omni") return drive(m.dat);
    if(uri == "/fb") return getInfo(m.dat);

    return -1;
  }

  //TODO: Error catching
  int DriveProtocol::drive(json dat) {
    tuum::cmds::drive(dat["s"], dat["d"].get<float>() / 1000.0, dat["r"]);
  }

  int DriveProtocol::getInfo(const json& dat) {
    json out;

    tuum::Motion *ptr = nullptr; // (Motion*)tuum::gSystem->findSubsystem(tuum::Motion::GetType());

    if(ptr != nullptr)
    {
      ptr->toJSON(out);
    }

    send(out);
  }

}
