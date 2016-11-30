
#include "rtex.hpp"
#include "tuum_context.hpp"

#include "STM.hpp"
#include "rtx_ctl.hpp"

using namespace tuum;

namespace rtex {

  STM* stm;

  void setup() {
    stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm->createState("STInit");
    stm->setState(st);
    ctx.st = st;
    st->addController(new ctl::LSInit(ctx));

    st2 = stm->createState("STBallLocate");
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSBallLocate(ctx));

    st2 = stm->createState("STBallNavigator");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSBallNavigator(ctx));

    //stm->setup();
  }

  void process() {
    //stm->process();
  }

}
