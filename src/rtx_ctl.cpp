/** @file rtx_ctl.hpp
 *  Tuum Robotex logic controllers implementation.
 *
 *  @authors Meelik Kiik
 *  @version 0.1
 *  @date 17. November 2015
 */

#include <boost/bind.hpp>

#include "tuum_visioning.hpp"
#include "tuum_localization.hpp"
#include "tuum_navigator.hpp"
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "MainBoard.hpp"

#include "hal.hpp"

#include "core/rtx_GameField.hpp"

#include "rtx_ctl.hpp"

#include "robotex.hpp"

using namespace tuum::comm;
using namespace tuum::hal;

namespace rtx {

  void autoaim()
  {
    if(gNav != nullptr)
    {

      // gNav->aim(b->getTransform()->getPosition());

      /*
      auto ctx = gNav->getContext();
      if(!ctx.hasAim())
      {
        GoalHandle gl = gGameField->getOpponentGoal();
        BallHandle bl = b; // gGameField->getNearestBall();

        if((gl != nullptr) && (bl != nullptr))
        {
          auto p1 = bl->getTransform()->getPosition(),
               p2 = gl->getTransform()->getPosition();

          Vec2f dv = Vec2f(p2 - p1) * 0.5;
          Vec2i p = p1 + dv;

          gNav->aim(p);
          printf("AUTOAIM ball=(%i, %i), goal=(%i, %i), dv=(%.1f, %.1f), aim=(%i, %i)\n", p1.x, p1.y, p2.x, p2.y, dv.x, dv.y, p.x, p.y);
          // ctx = gNav->getContext();
        }
        */
      }
  }

  void TwitchScan::_init() {
    wait_for_vision = true;

    motionTimer.setPeriod(160);
    visionTimer.setPeriod(600);

    motionTimer.start();
    visionTimer.start();
  }

  void TwitchScan::init() {
    _init();
  }

  void TwitchScan::init(int sp_vision, int sp_scan) {
    m_spv = sp_vision;
    m_sps = sp_scan;
    _init();
  }

  void TwitchScan::run() {
    if(!wait_for_vision) {
      if(motionTimer.isTime()) {
        hal::hw.getMotionControl()->omniDrive(0, 0, 90);

        wait_for_vision = true;
        visionTimer.start();
      }
    } else {
      if(visionTimer.isTime()) {
        hal::hw.getMotionControl()->omniDrive(0, 0, 0);

        wait_for_vision = false;
        motionTimer.start();
      }
    }
  }

}

namespace rtx {

  hal::MainBoard* mb = hal::hw.getMainBoard();

  // Ball search
  void LSPlaceholder::init() {
    mTmr.start(1000);

    RefereeListener* ref = hal::hw.getRefListener();

    /*ref->registerCallback(REF_KICKOFF, [this](RefCommand rcmd){
      this->kickOff();
    });*/

  }

  /*void LSPlaceholder::deinit() {
    RefereeListener* ref = hal::hw.getRefListener();

    ref->deregisterCallback(REF_KICKOFF, [this](RefCommand rcmd){
      this->kickOff();
    });
  }*/

  /*void kickOff() {
    mDoKickoff = true;
  }*/

  int LSPlaceholder::run() {
    if(!mDoKickoff) return 0;

    if(display_name) {
      printf("%s\n", mName.c_str());
      display_name = false;

    }

    if(!mTmr.isTime()) {
      hal::hw.getMotorControl()->omniDrive(mSpeed, mRad, mDeg);
    } else {
      hal::hw.getMotorControl()->omniDrive(0, 0, 0);
    }

    return 0;
  }

  bool LSPlaceholder::isRunnable() {
    return true;
  }


  bool LSPlaceholder::isInterruptable() {
    return true;
  }


  // Warmup
  bool LSInit::isRunnable() {
    return true;
  }

  int LSInit::run() {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());

    if(gNav == nullptr)
    {
      RTXLOG("Navigator subsystem handle request failed!", LOG_ERR);
      return -1;
    }

    switch(ctx.phase) {
      case CP_INIT:
        ctx.phase = CP_RUN;
        break;
      case CP_RUN:
        // TODO: timer/motion timeout/finish check
        ctx.phase = CP_DONE;
        break;
      case CP_DONE:
        break;
    }

    return 0;
  }

  bool LSInit::isInterruptable() {
    return ctx.phase == CP_DONE;
  }


  // Ball search
  void LSBallLocate::init() {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());

    if(gNav != nullptr) gNav->stop();

    twitchScanner.init(5, 30);

    mb->initialState();
  }

  int LSBallLocate::run() {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());
    if(gNav == nullptr)
    {
      printf("[LSBallLocate]Navigator handle request failed!\n");
      return 0;
    }

    if(gGameField->countValidBalls() > 0)
    {
      return 0;
    }
    else
    {
      twitchScanner.run();
    }

    return 0;
  }

  bool LSBallLocate::isRunnable() {
    return true;
  }


  // Navigate to ball
  void LSBallNavigator::init() {
    BallHandle b = gGameField->getNearestBall();

    if(b != nullptr)
      printf("[LSBallNavigator]Begin nav to %s\n", b->toString().c_str());

    m_dbg_clk.init(1000);
  }

  int LSBallNavigator::run()
  {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());

    BallHandle bl;
    GoalHandle gl;

    // if(mb->getBallSensorState()) goto OK;
    if(gGameField->countValidBalls() <= 0) goto ERR;

    bl = gGameField->getNearestBall();
    gl = gGameField->getOpponentGoal();

    if(bl != nullptr) {
      Transform t = gGameField->ballPickupPos(bl, gl);
      Vec2i apos;

      if(gl) apos = gl->getTransform()->getPosition();
      else apos = bl->getTransform()->getPosition();

      gNav->navTo(t.getPosition());

      if(gl)
      {
        apos = gl->getTransform()->getPosition();
        gNav->aim(apos);
      } else apos = bl->getTransform()->getPosition();

      if(!gNav->isTargetAchieved()) {
        //Deprecated: if(!gNav->isRunning()) gNav->start();
        if(m_dbg_clk.tick())
        {
          printf("[LSBallNavigator]Navigate to %s. Aim at (%i, %i)\n", bl->toString().c_str(), apos.x, apos.y);
        }
      } else {
        goto OK;
      }

    } else {
      if(gNav != nullptr)
      {
        gNav->navTo();
        gNav->aim();
        gNav->stop();
      }
    }

    return 0;
OK:
    if(gNav != nullptr) gNav->stop();
    return 1;
ERR:
    if(gNav != nullptr) gNav->stop();
    return -1;
  }

  bool LSBallNavigator::isRunnable() {
    //  || mb->getBallSensorState()
    return gGameField->countValidBalls() > 0;
  }


  // Ball pickup
  void LSBallPicker::init() {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());
    if(gNav != nullptr) gNav->stop();

    mb->pickupState();
  }

  int LSBallPicker::run() {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());

    BallHandle b = nullptr;

    // if(mb->getBallSensorState()) goto OK;
    if(gGameField->countValidBalls() <= 0) goto ERR;

    b = gGameField->getNearestBall();

    if(b != nullptr)
    {
      const double dD = Motion::DribblerPlanePadding;

      Transform* t = b->getTransform();
      Transform* me = Localization::getTransform();

      Vec2f avf = (t->getPosition() - me->getPosition()).getNormalized();

      if(gNav != nullptr) gNav->stop();

      if(m_dbg_clk.tick())
      {
        printf("[LSBallPicker]#TODO: Set throw distance\n");
      }

      //gNav->navTo(t->getPosition() + (Vec2i)(avf*dD));
      //gNav->aim(t->getPosition() + (Vec2i)(avf*1.1*dD));

      // if(me->getPosition().distanceTo(t->getPosition()) > dD) return -1;

      //if(!mb->getDribblerState()) mb->startDribbler(0.4);
      // if(!gNav->isRunning()) gNav->start();
    } else {
      if(gNav != nullptr) gNav->stop();
    }

    return 0;
OK:
    if(gNav != nullptr) gNav->stop();
    return 1;
ERR:
    if(gNav != nullptr) gNav->stop();
    //mb->stopDribbler();
    return -1;
  }

  bool LSBallPicker::isRunnable() {
    if(mb->getBallSensorState()) return true;

    BallHandle b = gGameField->getNearestBall();
    if(b == nullptr) return false;

    Transform* t = Localization::getTransform();
    double d = t->distanceTo(b->getTransform()->getPosition());
    if(d > Motion::MinDist) return false;

    return true;
  }


  // Opposing goal search
  bool LSGoalLocate::isRunnable() {
    if(mb->getBallSensorState()) {
      if(!mb->getDribblerState()) mb->startDribbler(0.4);
      return true;
    }
    return false;
  }

  void LSGoalLocate::init() {
    gNav->stop();
    ctx.phase = CP_INIT;
    twitchScanner.init(10, 30);
    if(!mb->getDribblerState()) mb->startDribbler(0.4);
  }

  int LSGoalLocate::run() {
    if(!mb->getBallSensorState()) goto ERR;
    if(gGameField->getOpponentGoal() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    gNav->stop();
    return 1;
ERR:
    gNav->stop();
    return -1;
  }


  ////////////////////
  void LSAllyGoalLocate::init() {
    gNav->stop();
    ctx.phase = CP_INIT;
    twitchScanner.init(10, 30);
  }

  int LSAllyGoalLocate::run() {
    if(gGameField->getAllyGoal() != nullptr) return 1;

    twitchScanner.run();

    return 0;
  }

  void LSAllyGoalMove::init() {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());
    if(gNav != nullptr) gNav->stop();

    GoalHandle goal = gGameField->getAllyGoal();
    if(goal != nullptr)
      std::cout << "Navigate to " << goal->toString() << std::endl;
  }

  int LSAllyGoalMove::run() {
    tuum::Navigator *gNav = (tuum::Navigator*)gSystem->findSubsystem(Navigator::GetType());
    Goal* goal = nullptr;

    if(goal != nullptr) {
      Vec2i pos = gGameField->calcAllyGoalPos(goal->getTransform()).getPosition();

      if(gNav != nullptr)
      {
        gNav->navTo(pos);
        gNav->aim(goal->getTransform()->getPosition());

        //std::cout << d << std::endl;
        //if(d < 250) mb->startDribbler();
        //else mb->stopDribbler();

        if(!gNav->isTargetAchieved()) {
          //if(!gNav->isRunning()) gNav->start();
        } else {
          goto OK;
        }
      }
    } else {
      if(gNav != nullptr) gNav->stop();
    }

    return 0;
OK:
    if(gNav != nullptr) gNav->stop();
    return 1;
ERR:
    if(gNav != nullptr) gNav->stop();
    return -1;
  }

  bool LSAllyGoalMove::isRunnable() {
    return true;
  }
  ////////////////////////////


  // Shoot to opposing goal
  void LSGoalShoot::init() {
    gNav->stop();
  }

  int LSGoalShoot::run() {
    if(!mb->getBallSensorState()) return -1;

    Goal* g = nullptr; // gGameField->getOpponentGoal();
    if(g == nullptr) return -1;

    gNav->navTo();
    gNav->aim(g->getTransform()->getPosition());

    if(!gNav->isTargetAchieved()) {
      // if(!gNav->isRunning()) gNav->start();
    } else {
      gNav->stop();
      if(mb->getBallSensorState()) mb->coilKick();
    }

    return 0;
  }

  bool LSGoalShoot::isRunnable() {
    return gGameField->getOpponentGoal() != nullptr && mb->getBallSensorState();
  }

  // Defend goal
  void LSGoalee::init() {
    gNav->stop();
  }

  int LSGoalee::run() {
    BallHandle b = gGameField->getNearestBall();

    if(b != nullptr) {
      Vec2i pos = gGameField->calcBallPickupPos(b->getTransform()).getPosition();

      gNav->navTo(pos);
      gNav->aim(b->getTransform()->getPosition());

      Transform* t = Localization::getTransform();
      double d = t->distanceTo(b->getTransform()->getPosition());

      //std::cout << d << std::endl;
      //if(d < 250) mb->startDribbler();
      //else mb->stopDribbler();

      if(!gNav->isTargetAchieved()) {
        //if(!gNav->isRunning()) gNav->start();
      }
    } else {
      gNav->stop();
    }

    return 0;
  }

  /**
   *
   *  Team interaction logic controllers
   *
   */
  bool LSAllyFind::isRunnable() {
    return true;
  }

  void LSAllyFind::init() {
    gNav->stop();
    twitchScanner.init(10, 30);
  }

  int LSAllyFind::run() {
    if(gGameField->getAlly() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    gNav->stop();
    return 1;
ERR:
    gNav->stop();
    return -1;
  }


  bool LSAllyLocate::isRunnable() {
    if(mb->getBallSensorState()) return true;
    return false;
  }

  void LSAllyLocate::init() {
    gNav->stop();
    twitchScanner.init(10, 30);
  }

  int LSAllyLocate::run() {
    if(gGameField->getAlly() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    gNav->stop();
    return 1;
ERR:
    gNav->stop();
    return -1;
  }


  bool LSAllyAim::isRunnable() {
    return gGameField->getAlly() != nullptr;
  }

  void LSAllyAim::init() {
    gNav->stop();
  }

  int LSAllyAim::run() {
    if(gGameField->getAlly() == nullptr) goto ERR;

    gNav->aim(gGameField->getAlly()->getTransform()->getPosition());

    return 0;
OK:
    gNav->stop();
    return 1;
ERR:
    gNav->stop();
    return -1;
  }


  // Pass ball to ally
  void LSAllyPass::init() {
    gNav->stop();
    commTimeout.setPeriod(5000);
    finish = false;
  }

  int LSAllyPass::run() {
    if(finish) return 0;

    if(mb->getBallSensorState()) {
      if(!mb->getDribblerState()) mb->startDribbler(0.4);
    } else {
      //FIXME: Ball lost Tuum signal?
      return -1;
    }

    if(commTimeout.isTime()) {
      tms = TuumMessage::toAlly(TuumSignal::PASS);
      hal::hw.getRefListener()->sendTuumMessage(tms);
      commTimeout.start();
    } else {
      if(comm::pollResponse(tms.id)) {
        tms = comm::popResponse(tms.id);
        finish = true;
        gNav->stop();

        MainBoard* mb = hal::hw.getMainBoard();

        mb->stopDribbler();
        mb->releaseCoil();

        emit("done");
      }
    }
    return 0;
  }

  bool LSAllyPass::isRunnable() {
    if(finish) return true;

    if(gGameField->getAlly() == nullptr) return false;
    if(!mb->getBallSensorState()) return false;
    return true;
  }


  // Receive ball from ally
  void LSAllyReceive::init() {
    gNav->stop();
    finish = false;

    auto cb = std::bind1st(std::mem_fun(&LSAllyReceive::onPassSignal), this);
    comm::registerListener(TuumSignal::PASS, cb);

    addHandler("STExit", [](){
      comm::deregisterListener(TuumSignal::PASS);
    });
  }

  int LSAllyReceive::run() {
    if(finish) return 0;

    return 0;
  }

  bool LSAllyReceive::isRunnable() {
    if(finish) return true;

    return true;
  }

  void LSAllyReceive::onPassSignal(TuumMessage tms) {
    std::cout << "LSAllyReceive DONE" << std::endl;
    emit("LSDone");
    finish = true;
  }

  void LSPlacedBallInit::init() {
    gNav->stop();
  }

  int LSPlacedBallInit::run() {
    //Move to ball until 500mm away

    /*BallHandle b = gNavigation->getNearestBall();

    if(b != nullptr) {
      Vec2i pos = gNavigation->calcPerimeterPosition(b->getTransform(), 500).getPosition();

      Motion::setPositionTarget(pos);
      Motion::setAimTarget(b->getTransform()->getPosition());

      Transform* t = Localization::getTransform();
      double d = t->distanceTo(b->getTransform()->getPosition());

      if(d > 500) {
        if(!Motion::isRunning()) Motion::start();
      } else {
        Motion::stop();
      }
    } else {
      Motion::stop();
    }*/

    return 1;
  }

  bool LSPlacedBallInit::isRunnable() {
    return true;
  }

  void LSWaitForEnemyKickoff::init() {
    gNav->stop();
    kickoffTimer.setPeriod(5000);
  }

  int LSWaitForEnemyKickoff::run() {
    kickoffTimer.start();

    while(!gameStarted) {

      if (kickoffTimer.isTime()) {
        gameStarted = true;
      }
      /*else if(the ball has moved more than 10cm){
        gameStarted = true;
      }*/
    }

  }

  void LSMoveToEntity::init() {
    //Scan for entity
  }

  int LSMoveToEntity::run() {

    /*if(e != nullptr) {
      Vec2i pos = gNavigation->calcPerimeterPosition(e->getTransform(), d).getPosition();

      Motion::setPositionTarget(pos);
      Motion::setAimTarget(e->getTransform()->getPosition());

      Transform* t = Localization::getTransform();
      double currentDistance = t->distanceTo(e->getTransform()->getPosition());

      if(currentDistance > d) {
        if(!Motion::isRunning()){
           Motion::start();
      } else {
        Motion::stop();
      }
    } else {
      Motion::stop();
    }
  }*/

    return 1;
  }

}
