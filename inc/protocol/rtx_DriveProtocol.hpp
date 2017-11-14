
#ifndef RTEX_DRVPROTO_H
#define RTEX_DRVPROTO_H

#include "tuum_wsproto.hpp"

using namespace tuum::wsocs;

namespace rtx {

  class DriveProtocol : public WSProtocol
  {
  public:
    DriveProtocol();

    int route(const WSProtocol::Message&);

    int drive(json);

    int getInfo(const json&);

  private:

  };

}

#endif
