
#include "hal.hpp"

#include "RobotexCommSrv.hpp"

#include "rtx_cmds.hpp"

using namespace tuum::wsocs;

namespace tuum { namespace gui {

  RobotexCommSrv::RobotexCommSrv():
    WebSocketServer()
  {
    mDrvProtocol.setWS(this);
    mVisProtocol.setWS(this);

    size_t id = proto()->add(mDrvProtocol.getDescriptor());
    id = proto()->add(mVisProtocol.getDescriptor());
    id = proto()->add(mHwProtocol.getDescriptor());
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
    auto it = ms.dat.find(WSProtocol::JS_URI);
    if(it == ms.dat.end()) return;
    if(!it.value().is_string()) return;

    mCtx->mId = ms.dat[WSProtocol::JS_M_ID].get<size_t>();

    if(proto()->route(ms) < 0) {
      RTXLOG(format("Unknown URI '%s'!", ms.getURI().c_str()), LOG_ERR);
    }
  }

}}
