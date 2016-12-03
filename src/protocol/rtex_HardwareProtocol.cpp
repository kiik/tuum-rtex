
#include "hal.hpp"

#include "rtx_cmds.hpp"
#include "protocol/rtex_HardwareProtocol.hpp"

using namespace tuum::wsocs;

namespace tuum {

  HardwareProtocol::HardwareProtocol() {
    mRouteMap["dr"] = ERoute::SetDribbler;
    mRouteMap["ch"] = ERoute::DoCoilCharge;
    mRouteMap["kc"] = ERoute::DoCoilKick;
    mRouteMap["bl"] = ERoute::GetBallSensor;
  }

  WSProtocol::route_t HardwareProtocol::getDescriptor()
  {
    WSProtocol::route_t out;
    out.uri = "/hw";
    out.wsp = this;
    return out;
  }

  int HardwareProtocol::route(const WSProtocol::Message& m) {
    ERoute r = ERoute::None;
    for(auto it = mRouteMap.begin(); it != mRouteMap.end(); it++) {
      if(it->first == m.dat[WSProtocol::JS_CMD]) {
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
