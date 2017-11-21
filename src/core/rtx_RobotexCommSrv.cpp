
#include "hal.hpp"

#include "core/rtx_RobotexCommSrv.hpp"

#include "rtx_cmd.hpp"

using namespace tuum::wsocs;

namespace rtx {

  RobotexCommSrv::RobotexCommSrv():
    WebSocketServer(),
    mDrvProtocol(new DriveProtocol()),
    mVisProtocol(new VisionProtocol()),
    mHwProtocol(new HardwareProtocol())
  {
    size_t id;
    id = registerProtocol(mDrvProtocol);
    id = registerProtocol(mVisProtocol);
    id = registerProtocol(mHwProtocol);
  }

  void RobotexCommSrv::onGet()
  {

  }

  void RobotexCommSrv::onConnect()
  {

  }

  int RobotexCommSrv::send(json& dat)
  {
    dat["_"] = mCtx->mId;
    return WebSocketServer::send(dat);
  }

  void RobotexCommSrv::onMessage(lws *wsi, void *in, size_t len)
  {
    WSProtocol::Request* req = (WSProtocol::Request*)in;

    switch(req->cmd) {
      case ECommand::URI:
        printf("[WSUI]TODO: URI - ResourceNetwork requests\n");
        break;
      default:
        if((req->cmd != ECommand::None) && (req->cmd < ECommand::CMD_N)) {
          printf("[WSUI]Warning: unhandled command '%i'!\n", req->cmd);
        }
        break;
    }
  }

  void RobotexCommSrv::onMessage(WSProtocol::Message ms)
  {
    // printf("%s\n", ms.dat.dump().c_str());

    mCtx->mId = ms.dat[WSProtocol::JS_M_ID].get<size_t>();

    if(proto()->route(ms) < 0) {
      RTXLOG(tuum::format("Unknown URI '%s'!", ms.getURI().c_str()), LOG_ERR);
    }
  }

}
