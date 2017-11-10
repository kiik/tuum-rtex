
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>

#include "base64.hpp"

#include "hal.hpp"

#include "tuum_system.hpp"
#include "tuum_visioning.hpp"
#include "tuum_context.hpp"

#include "lpx_iformat.hpp"

#include "protocol/rtex_VisionProtocol.hpp"

using namespace tuum::wsocs;

namespace tuum {

  const char* JS_METHOD = "m";
  const char* JS_MTH_GET = "get";
  const char* JS_MTH_PUT = "put";
  const char* JS_DATA = "data";


  VisionProtocol::VisionProtocol():
    WSProtocol({
      "Vision Protocol",
      "/vis",
      "0.0.1-al.0",
      {
        {"Get Frame", "/gf", {}},

        {"Config", "/cnf", {
            {"Threshold En", "threshold", WSType::WST_String},
          }
        },

        {"Set thresholding", "/thr", {
          {"Threshold En", "threshold", WSType::WST_Integer},
        }},


        {"Filters", "/flt", {}},
      },
      this
    })
  {

  }

  int VisionProtocol::route(const WSProtocol::Message& m) {
    std::string cmd = m.dat[WSProtocol::JS_URI].get<std::string>();

    if(cmd == "/gf") return getFrame(m.dat);
    if(cmd == "/thr") return toggleThresholding(m.dat);

    if(cmd == "/flt") {
      if(m.dat[JS_METHOD].get<std::string>() == JS_MTH_GET) {

      } else if(m.dat[JS_METHOD].get<std::string>() == JS_MTH_PUT) {

      }

      return 0;
    }

    if(cmd == "/cnf") return vConfig(m.dat);
    if(boost::starts_with(cmd, "vf_")) return vFilter(m.dat);
    if(cmd == "/ent_get") return getEntities();

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

  int VisionProtocol::vConfig(const json& dat) {
    if(dat.find("data") != dat.end())
      tuum::gVision->configure(dat["data"]);

    json out;
    tuum::gVision->toJSON(out);
    send(out);

    return 0;
  }

  int VisionProtocol::vFilter(const json& dat) {
    std::string cmd = dat[WSProtocol::JS_URI].get<std::string>();

    if(boost::ends_with(cmd, "get")) {
      json res;
      tuum::gVision->getFilter()->toJSON(res);
      res["_r"] = "OK";
      send(res);
    } else if(boost::ends_with(cmd, "set")) {
      auto r = dat["f"]["range"];
      VisionFilter::ColorClass cls = {r[0], r[1], r[2], r[3], r[4], r[5]};
      cls.id = dat["f"]["id"];
      printf("SET %s\n", dat.dump().c_str());
      tuum::gVision->getFilter()->updateYUVClassifier(cls);

      json res;
      res["_r"] = "OK";
      send(res);
    }

    return 0;
  }

  int VisionProtocol::getEntities() {
    json dat;
    //FIXME: tuum::gSystem->getEntityHandle()->toJSON(dat);
    send(dat);
    return 0;
  }

}
