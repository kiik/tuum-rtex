/** @file LogicManager.cpp
 *  Logic machines loading class implementation.
 *
 *  @authors Meelik Kiik
 *  @version 0.1
 *  @date 28. November 2015
 */

#include "tuum_navigator.hpp"
#include "tuum_context.hpp"

#include "core/rtx_LogicManager.hpp"

#include "rtx_ctl.hpp"

namespace rtx {

  /**
   *  Controllers:
   *    LSBallScan: Find ball
   *    LSBallKickPrepare: Move in front of ball
   */
  STM* LogicManager::loadKickoffPasserPrepare() {
    STM* stm = new STM();
    State *st, *st2;
    Context ctx;

    st = stm->createState("STBallLocate");
    ctx.st = st;
    st->addController(new ctl::LSBallLocate(ctx));

    st2 = stm->createState("STBallNavigator");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSBallNavigator(ctx));

    return stm;
  }

  STM* LogicManager::loadKickoffReceiverPrepare() {
    STM* stm = new STM();
    State *st, *st2;
    Context ctx;

    st = stm->createState("STAllyFind");
    ctx.st = st;
    st->addController(new ctl::LSAllyFind(ctx));


    st2 = stm->createState("STAllyAim");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSAllyAim(ctx));

    return stm;
  }


  /**
   *  Controllers:
   *    LSAllyLocate: Find ally robot
   *    LSAllyPass: Pass to ally robot
   */
  STM* LogicManager::loadKickoffPasser() {
    STM* stm = LogicManager::loadKickoffPasserPrepare();
    State *st2, *st = stm->stateStackPeek();
    Context ctx;

    st2 = stm->createState("STBallPicker");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSBallPicker(ctx));

    st2 = stm->createState("STAllyLocate");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSAllyLocate(ctx));

    st2 = stm->createState("STAllyPass");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSAllyPass(ctx));

    return stm;
  }


  /**
   *  Controllers:
   *    LSAllyLocate: Find ally robot
   *    LSAllyRecv: Pass to ally robot
   */
  STM* LogicManager::loadKickoffReceiver() {
    STM* stm = LogicManager::loadKickoffReceiverPrepare();
    State *st2, *st = stm->stateStackPeek();
    Context ctx;

    return stm;
  }


  /**
   *  Controllers:
   *    LSBallLocate: Find ball
   *    LSBallNavigator: Move in front of ball
   *    LSBallPicker: Pickup ball
   *    LSGoalLocate (+rootstate): Find opposing goal
   *    LSGoalShoot: Aim & kick at goal
   */
  STM* LogicManager::loadOffensivePlay() {
    STM* stm = new STM();
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


    st2 = stm->createState("STBallPicker");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSBallPicker(ctx));

    st2 = stm->createState("STGoalLocate");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSGoalLocate(ctx));
    stm->addRootState(st);


    st2 = stm->createState("STGoalShoot");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSGoalShoot(ctx));

    return stm;
  }

  ///////////////////////

  STM* LogicManager::loadDefensivePlay()
  {
    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm->createState("STFindAllyGoal");
    stm->setState(st);
    ctx.st = st;
    st->addController(new ctl::LSAllyGoalLocate(ctx));

    st2 = stm->createState("STAllyGoalMove");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSMoveToEntity(ctx, nullptr, 100)); // FIXME: gNavi->getAllyGoal()

    st2 = stm->createState("STGoalee");
    stm->setState(st);
    ctx.st = st;
    st->addController(new ctl::LSGoalee(ctx));

    return stm;
  }

  ////////////////////////////////////////////

  STM* LogicManager::placedBall()
  {

    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm->createState("STBallLocate");
    stm->setState(st);
    ctx.st = st;
    st->addController(new ctl::LSBallLocate(ctx));


    st2 = stm -> createState("STMoveToBall");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    //st->addController(new ctl::LSMoveToEntity(ctx, gNavi->getNearestBall(), 500));

    std::cout << "State placed ball" << std::endl;

    return stm;
  }

  STM* LogicManager::enemyKikcoffPrepare()
  {
    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm -> createState("STMoveToBall");
    stm->setState(st);
    ctx.st = st;
    //st->addController(new ctl::LSMoveToEntity(ctx, gNavi->getNearestBall(), 1000));

    return stm;
  }

  STM* LogicManager::enemyKikcoff()
  {
    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm -> createState("STWaitForEnemyKickoff");
    stm->setState(st);
    ctx.st = st;
    st ->addController(new ctl::LSWaitForEnemyKickoff(ctx));

    //Move to centerline

    /*st = stm-> createState("STPlaceholder");
    //st2->setLastState(st);
    //st->setNextState(st2);
    //st = st2;
    ctx.st = st;
    st->addController(new ctl::LSPlaceholder(ctx, "LSPlaceholder#0", 10, 0, 0));


    st2 = stm-> createState("STPlaceholder2");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSPlaceholder(ctx, "LSPlaceholder#1", 0, 0, 10));*/

    return stm;
  }

  STM* LogicManager::stmPlaceholder()
  {
    /*
    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm->createState("STBallLocate");
    stm->setState(st);
    ctx.st = st;
    st->addController(new ctl::LSBallLocate(ctx));


    st2 = stm->createState("STMoveToBall");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSBallNavigator(ctx));*/

    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    //Move to centerline

    st = stm-> createState("STPlaceholder");
    //st2->setLastState(st);
    //st->setNextState(st2);
    //st = st2;
    ctx.st = st;
    st->addController(new ctl::LSPlaceholder(ctx, "LSPlaceholder#0", 0, 10, 0));


    st2 = stm-> createState("STPlaceholder2");
    st2->setLastState(st);
    st->setNextState(st2);
    st = st2;
    ctx.st = st;
    st->addController(new ctl::LSPlaceholder(ctx, "LSPlaceholder#1", 10, 10, 0));

    return stm;

  }

  STM* LogicManager::throwinPrepare()
  {
    STM* stm = new STM();
    State* st, *st2;
    Context ctx;

    st = stm -> createState("STMoveToBall");
    stm->setState(st);
    ctx.st = st;
    // st->addController(new ctl::LSMoveToEntity(ctx, gNavi->getNearestBall(), 500));

    return stm;
  }
}
