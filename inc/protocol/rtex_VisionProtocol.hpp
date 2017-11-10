
#ifndef RTX_VISION_PROTOCOL_H
#define RTX_VISION_PROTOCOL_H

#include "tuum_wsproto.hpp"

using namespace tuum::wsocs;

namespace tuum {

  class VisionProtocol : public WSProtocol
  {
  public:
    VisionProtocol();

    int route(const WSProtocol::Message&);

    int getFrame(const json&);

    int toggleThresholding(const json&);
    int vConfig(const json&);
    int vFilter(const json&);

    int getEntities();

  private:

  };

}

#endif
