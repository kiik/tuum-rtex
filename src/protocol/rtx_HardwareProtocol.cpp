
#include "hal.hpp"

#include "protocol/rtx_HardwareProtocol.hpp"

#include "rtx_cmd.hpp"

using namespace tuum;
using namespace tuum::wsocs;

namespace rtx {

  HardwareProtocol::HardwareProtocol():
    WSProtocol({
      "Hardware Protocol",
      "/hw",
      "0.0.1-al.0",
      {
        {"Set Dribbler", "/dr", {
            {"Power", "v", WSType::WST_Integer},
          }
        },
        {"Coil Charge", "/ch", {}},
        {"Coil Kick", "/kc", {}},
        {"Get Ball Sensor", "/bl", {}},
      },
      this
    })
  {
    mRouteMap["/dr"] = ERoute::SetDribbler;
    mRouteMap["/ch"] = ERoute::DoCoilCharge;
    mRouteMap["/kc"] = ERoute::DoCoilKick;
    mRouteMap["/bl"] = ERoute::GetBallSensor;
  }

  int HardwareProtocol::route(const WSProtocol::Message& m) {
    ERoute r = ERoute::None;
    for(auto it = mRouteMap.begin(); it != mRouteMap.end(); it++) {
      if(it->first == m.dat[WSProtocol::JS_URI]) {
        r = it->second;
        break;
      }
    }

    if(r == ERoute::None) return -1;

    switch(r) {
      case ERoute::SetDribbler:
        return setDribbler(m.dat);
      case ERoute::DoCoilCharge:
        return doCoilCharge();
      case ERoute::DoCoilKick:
        return doCoilKick(m.dat);
      case ERoute::GetBallSensor:
        return getBallSensor();
    }

    return -2;
  }

  int HardwareProtocol::setDribbler(const json& dat) {
    uint8_t v = 0;
    if(dat.find("v") != dat.end()) v = dat["v"].get<uint8_t>();

    auto mb = hal::hw.getMainBoard();
    if(v == 0) mb->startDribbler();
    else mb->stopDribbler();

    json res = {"_r", "OK"};
    send(res);
    return 0;
  }

  int HardwareProtocol::doCoilCharge() {
    hal::hw.getMainBoard()->chargeCoil();

    json res = {"_r", "OK"};
    send(res);
    return 0;
  }

  int HardwareProtocol::doCoilKick(const json& dat) {
    hal::hw.getMainBoard()->releaseCoil();

    json res = {"_r", "OK"};
    send(res);
    return 0;
  }

  int HardwareProtocol::getBallSensor() {
    uint8_t v = 0;
    v = hal::hw.getMainBoard()->getBallSensorState();

    json res = {{"_r", "OK"}, {"v", v}};
    send(res);
    return 0;
  }

}
