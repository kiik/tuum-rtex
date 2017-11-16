
#include "STM.hpp"
#include "RefereeListener.hpp"

#include "rtx_ctl.hpp"

#include "hal.hpp"

#include "game/rtx_basketball.hpp"

using namespace tuum;
using namespace tuum::hal;

namespace rtx {
namespace Basketball {

  enum GameState {
    STOP,
    RUN,
  } gameState = STOP;

  STM *lg_offense;

  STM *activeLogic = nullptr;


  void loadLogic();
  void setupRefSignals();

  void pause();
  void resume();


  void setup()
  {
    loadLogic();
    setupRefSignals();

    activeLogic = lg_offense;

    resume(); // Bypass Ref module
  }

  void process()
  {
    switch(gameState) {
      case GameState::STOP:
        break;
      case GameState::RUN:
        if(activeLogic != nullptr) activeLogic->process();
        break;
      default:
        break;
    }
  }


  void pause()
  {
    std::cout << "PAUSE" << std::endl;
    gameState = GameState::STOP;

    // hw.getMainBoard()->stopDribbler();
  }

  void resume()
  {
    std::cout << "RESUME" << std::endl;
    gameState = GameState::RUN;
  }


  void loadLogic()
  {
    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm->createState("STInit");
    stm->setState(st);
    ctx.st = st;
    st->addController(new rtx::LSInit(ctx));

    st2 = stm->createState("STBallLocate");
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new rtx::LSBallLocate(ctx));

    st2 = stm->createState("STBallNavigator");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new rtx::LSBallNavigator(ctx));


    st2 = stm->createState("STBallPicker");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new rtx::LSBallPicker(ctx));

    st2 = stm->createState("STGoalLocate");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new rtx::LSGoalLocate(ctx));
    stm->addRootState(st);


    st2 = stm->createState("STGoalShoot");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new rtx::LSGoalShoot(ctx));

    lg_offense = stm;
  }

  void setupRefSignals()
  {
    RefereeListener* ref = hw.getRefListener();

    ref->registerCallback(REF_START, [=](RefCommand rcmd){
      resume();
    });

    ref->registerCallback(REF_STOP, [=](RefCommand rcmd){
      pause();
    });
  }

}}
