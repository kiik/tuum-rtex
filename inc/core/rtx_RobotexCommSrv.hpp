
#ifndef RTX_COMM_H
#define RTX_COMM_H

#include "json.hpp"

#include "tuum_wsproto.hpp"

#include "wsocs/WebSocketServer.hpp"

#include "protocol/rtx_VisionProtocol.hpp"
#include "protocol/rtx_DriveProtocol.hpp"
#include "protocol/rtx_HardwareProtocol.hpp"

using namespace tuum;
using namespace tuum::wsocs;

namespace rtx {

  class RobotexCommSrv : public wsocs::WebSocketServer
  {
  public:
    RobotexCommSrv();

    // Standard ABI
    enum ECommand {
      None    = 0,
      DBG     = 1,
      URI     = 2,

      CMD_N,
    };

  protected:

    void onGet();

    void onConnect();
    void onMessage(lws*, void*, size_t);
    void onMessage(WSProtocol::Message);

    int send(json& dat);

  private:
    DriveProtocol  mDrvProtocol;
    VisionProtocol mVisProtocol;
    HardwareProtocol mHwProtocol;

  };

}

#endif
