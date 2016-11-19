
#include "base64.hpp"

#include "hal.hpp"

#include "tuum_visioning.hpp"

#include "lpx_iformat.hpp"
#include "rtx_cmds.hpp"
#include "protocol/rtex_VisionProtocol.hpp"

using namespace tuum::wsocs;

namespace tuum {

  VisionProtocol::VisionProtocol() {

  }

  WSProtocol::route_t VisionProtocol::getDescriptor()
  {
    WSProtocol::route_t out;
    out.uri = "/vis";
    out.wsp = this;
    return out;
  }

  const char* JS_METHOD = "m";
  const char* JS_MTH_GET = "get";
  const char* JS_MTH_PUT = "put";
  const char* JS_DATA = "data";

  int VisionProtocol::route(const WSProtocol::Message& m) {
    std::string cmd = m.dat[WSProtocol::JS_CMD].get<std::string>();

    if(cmd == "getFrame") {
      return getFrame(m.dat);
    } else if (cmd == "settings") {
      return toggleThresholding(m.dat);
    } else if (cmd == "filters") {
      if(m.dat[JS_METHOD].get<std::string>() == JS_MTH_GET) {

      } else if(m.dat[JS_METHOD].get<std::string>() == JS_MTH_PUT) {

      }
    } else if(cmd =="pplcnf") {
      return pplConfig(m.dat[JS_DATA]);
    }

    return -1;
  }

  //TODO: Error catching
  int VisionProtocol::getFrame(const json& dat) {
    if(dat["dev"].get<std::string>() == "CAM0") {
      //hal::hw.
      json res;

      tuum::Visioning* vis = tuum::gVision;
      if(vis == nullptr) return -1;

      image_t img;
      Env::spawnBuffer(img);

      if(vis->readFrame(img) < 0) return -2;

      image_t out = lpx::rgb_to_png(img);

      std::string b64_img = b64::encode((const char*)out->data, out->size);
      res["frame"] = b64_img;

      send(res);
      return 0;
    }

    return -1;
  }

  int VisionProtocol::toggleThresholding(const json& dat) {
    if(dat["threshold"].get<bool>() == true)
      tuum::gVision->setThresholding(true);
    else
      tuum::gVision->setThresholding(false);
    return 0;
  }

  int VisionProtocol::pplConfig(const json& dat) {
    tuum::gVision->pplConfig(dat);
    return 0;
  }

}
