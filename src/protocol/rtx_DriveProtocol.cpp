
#include "tuum_motion.hpp"
#include "tuum_context.hpp"
#include "tuum_system.hpp"

#include "protocol/rtx_DriveProtocol.hpp"

#include "rtx_cmd.hpp"

using namespace tuum::wsocs;

namespace rtx {

  DriveProtocol::DriveProtocol():
    WSProtocol({
      "Drive Protocol",
      "/drv",
      "0.0.1-al.0",
      {
        {"Omni Drive", "/omni", {
            {"Motor Power", "s", WSType::WST_Integer},
            {"Direction",   "d", WSType::WST_Integer},
            {"Turn Rate",   "r", WSType::WST_Integer},
          }
        }
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
    int v = dat["s"].get<int>();
    float a = dat["d"].get<int>();
    int rot_v = dat["r"].get<int>();

    if(a == 1) a = 1.57;
    else if(a == -1) a = -1.57;
    else if(a == 3) a = 3.14;
    else a = 0;

    rtx::cmd::drive(v, a, rot_v);
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
