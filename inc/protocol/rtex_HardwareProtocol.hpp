
#ifndef RTEX_HWPROTO_H
#define RTEX_HWPROTO_H

#include <string>
#include <map>

#include "tuum_wsproto.hpp"

using namespace tuum::wsocs;

namespace tuum {

  class HardwareProtocol : public WSProtocol
  {
  public:
    enum ERoute {
      None,

      SetDribbler,

      DoCoilCharge,
      DoCoilKick,

      GetBallSensor,
    };

    HardwareProtocol();

    WSProtocol::route_t getDescriptor();

    int route(const WSProtocol::Message&);

    int setDribbler(const json&);
    int doCoilCharge();
    int doCoilKick(const json&);
    int getBallSensor();


  private:
    std::map<std::string,ERoute> mRouteMap;
  };

}

#endif
