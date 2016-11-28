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
#include "tuum_navigation.hpp"
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "hal.hpp"

#include "rtx_ctl.hpp"

using namespace tuum::comm;
using namespace tuum::hal;

namespace tuum {

  void TwitchScan::_init() {
    wait_for_vision = true;

    motionTimer.setPeriod(160);
    visionTimer.setPeriod(400);
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
        gMotion->setAimTarget(vec2i({0, 1}));
        //motionData.manualRotGear = {m_spv, 3.14};
        gMotion->start();
        wait_for_vision = true;
        visionTimer.start();
      }
    } else {
      if(visionTimer.isTime()) {
        gMotion->setAimTarget(vec2i({0, 1}));
        //motionData.manualRotGear = {m_sps, 3.14};
        gMotion->start();
        wait_for_vision = false;
        motionTimer.start();
      }
    }
  }

}

namespace tuum { namespace ctl {

  hal::MainBoard* mb = hal::hw.getMainBoard();


  // Warmup
  bool LSInit::isRunnable() {
    return true;
  }

  int LSInit::run() {
    switch(ctx.phase) {
      case CP_INIT:
        gMotion->setTarget({10, 10});

        ctx.phase = CP_RUN;
        break;
      case CP_RUN:
        // TODO: timer/motion timeout/finish check
        ctx.phase = CP_DONE;
        break;
      case CP_DONE:
        break;
    }
  }

  bool LSInit::isInterruptable() {
    return ctx.phase == CP_DONE;
  }


  // Ball search
  void LSBallLocate::init() {
    gMotion->stop();
    twitchScanner.init(5, 30);
    mb->stopDribbler();
  }

  int LSBallLocate::run() {
    if(gNavi->countValidBalls() > 0) {
      mb->startDribbler();
      gMotion->stop();
      return 0;
    } else {
      twitchScanner.run();
    }

    return 0;
  }

  bool LSBallLocate::isRunnable() {
    return true;
  }


  // Navigate to ball
  void LSBallNavigator::init() {
    gMotion->stop();

    Ball* b = gNavi->getNearestBall();
    if(b != nullptr)
      std::cout << "Navigate to " << b->toString() << std::endl;
  }

  int LSBallNavigator::run() {
    Ball* b = nullptr;
    if(mb->getBallSensorState()) goto OK;
    if(gNavi->countValidBalls() <= 0) goto ERR;

    b = gNavi->getNearestBall();

    if(b != nullptr) {
      vec2i pos = gNavi->calcBallPickupPos(b->getTransform()).getPosition();

      gMotion->setTarget(pos, b->getTransform()->getPosition());
      mb->startDribbler();

      //std::cout << d << std::endl;
      //if(d < 250) mb->startDribbler();
      //else mb->stopDribbler();

      if(!gMotion->isTargetAchieved()) {
        if(!gMotion->isRunning()) gMotion->start();
      } else {
        goto OK;
      }
    } else {
      gMotion->stop();
    }

    return 0;
OK:
    gMotion->stop();
    return 1;
ERR:
    gMotion->stop();
    return -1;
  }

  bool LSBallNavigator::isRunnable() {
    return gNavi->countValidBalls() > 0 || mb->getBallSensorState();
  }


  // Ball pickup
  void LSBallPicker::init() {
    gMotion->stop();
    mb->startDribbler();
  }

  int LSBallPicker::run() {
    Ball* b = nullptr;
    if(mb->getBallSensorState()) goto OK;
    if(gNavi->countValidBalls() <= 0) goto ERR;

    b = gNavi->getNearestBall();

    if(b != nullptr) {
      double dD = Motion::DribblerPlanePadding;
      Transform* t = b->getTransform();
      Transform* me = Localization::getTransform();
      vec2 avf = (t->getPosition() - me->getPosition()).getNormalized();
      gMotion->setTarget(t->getPosition() + (vec2i)(avf*dD), t->getPosition() + (vec2i)(avf*1.1*dD));

      if(me->getPosition().distanceTo(t->getPosition()) > dD) return -1;

      mb->startDribbler();
      if(!gMotion->isRunning()) gMotion->start();
    } else {
      gMotion->stop();
    }

    return 0;
OK:
    gMotion->stop();
    return 1;
ERR:
    gMotion->stop();
    mb->stopDribbler();
    return -1;
  }

  bool LSBallPicker::isRunnable() {
    if(mb->getBallSensorState()) return true;

    Ball* b = gNavi->getNearestBall();
    if(b == nullptr) return false;

    Transform* t = Localization::getTransform();
    double d = t->distanceTo(b->getTransform()->getPosition());
    if(d > Motion::MinDist) return false;

    return true;
  }


  // Opposing goal search
  bool LSGoalLocate::isRunnable() {
    if(mb->getBallSensorState()) {
      mb->startDribbler();
      return true;
    }
    return false;
  }

  void LSGoalLocate::init() {
    gMotion->stop();
    ctx.phase = CP_INIT;
    twitchScanner.init(10, 30);
    mb->startDribbler();
  }

  int LSGoalLocate::run() {
    if(!mb->getBallSensorState()) goto ERR;
    if(gNavi->getOpponentGoal() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    gMotion->stop();
    return 1;
ERR:
    gMotion->stop();
    return -1;
  }


  ////////////////////
  void LSAllyGoalLocate::init() {
    gMotion->stop();
    ctx.phase = CP_INIT;
    twitchScanner.init(10, 30);
  }

  int LSAllyGoalLocate::run() {
    if(gNavi->getAllyGoal() != nullptr) return 1;

    twitchScanner.run();

    return 0;
  }

  void LSAllyGoalMove::init() {
    gMotion->stop();

    Goal* goal = gNavi->getAllyGoal();
    if(goal != nullptr)
      std::cout << "Navigate to " << goal->toString() << std::endl;
  }

  int LSAllyGoalMove::run() {
    Goal* goal = nullptr;
    //if(värava ees) goto OK;
    //if(väravat ei leitud) goto ERR;

    if(goal != nullptr) {
      vec2i pos = gNavi->calcAllyGoalPos(goal->getTransform()).getPosition();

      gMotion->setTarget(pos, goal->getTransform()->getPosition());

      //std::cout << d << std::endl;
      //if(d < 250) mb->startDribbler();
      //else mb->stopDribbler();

      if(!gMotion->isTargetAchieved()) {
        if(!gMotion->isRunning()) gMotion->start();
      } else {
        goto OK;
      }
    } else {
      gMotion->stop();
    }

    return 0;
OK:
    gMotion->stop();
    return 1;
ERR:
    gMotion->stop();
    return -1;
  }

  bool LSAllyGoalMove::isRunnable() {
    return true;
  }
  ////////////////////////////


  // Shoot to opposing goal
  void LSGoalShoot::init() {
    gMotion->stop();
  }

  int LSGoalShoot::run() {
    if(!mb->getBallSensorState()) return -1;

    Goal* g = gNavi->getOpponentGoal();
    if(g == nullptr) return -1;

    //gMotion->setPositionTarget(gNavi->getGoalShootPosition(g));
    gMotion->setAimTarget(g->getTransform()->getPosition());
    //std::cout << g->getTransform()->getPosition().toString() << std::endl;;
;
    //if(fabs(gMotion->getDeltaOrientation()) < 0.030) mb->releaseCoil();

    if(!gMotion->isTargetAchieved()) {
      if(!gMotion->isRunning()) gMotion->start();
    } else {
      gMotion->stop();
      if(mb->getBallSensorState()) mb->releaseCoil();
    }

    return 0;
  }

  bool LSGoalShoot::isRunnable() {
    return gNavi->getOpponentGoal() != nullptr && mb->getBallSensorState();
  }

  // Defend goal
  void LSGoalee::init() {
    gMotion->stop();
  }

  int LSGoalee::run() {
    Ball* b = gNavi->getNearestBall();

    if(b != nullptr) {
      vec2i pos = gNavi->calcBallPickupPos(b->getTransform()).getPosition();

      gMotion->setTarget(pos, b->getTransform()->getPosition());

      Transform* t = Localization::getTransform();
      double d = t->distanceTo(b->getTransform()->getPosition());

      //std::cout << d << std::endl;
      //if(d < 250) mb->startDribbler();
      //else mb->stopDribbler();

      if(!gMotion->isTargetAchieved()) {
        if(!gMotion->isRunning()) gMotion->start();
      }
    } else {
      gMotion->stop();
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
    gMotion->stop();
    twitchScanner.init(10, 30);
  }

  int LSAllyFind::run() {
    if(gNavi->getAlly() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    gMotion->stop();
    return 1;
ERR:
    gMotion->stop();
    return -1;
  }


  bool LSAllyLocate::isRunnable() {
    if(mb->getBallSensorState()) return true;
    return false;
  }

  void LSAllyLocate::init() {
    gMotion->stop();
    twitchScanner.init(10, 30);
  }

  int LSAllyLocate::run() {
    if(gNavi->getAlly() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    gMotion->stop();
    return 1;
ERR:
    gMotion->stop();
    return -1;
  }


  bool LSAllyAim::isRunnable() {
    return gNavi->getAlly() != nullptr;
  }

  void LSAllyAim::init() {
    gMotion->stop();
  }

  int LSAllyAim::run() {
    if(gNavi->getAlly() == nullptr) goto ERR;

    gMotion->setAimTarget(gNavi->getAlly()->getTransform()->getPosition());
    if(!gMotion->isRunning()) gMotion->start();

    return 0;
OK:
    gMotion->stop();
    return 1;
ERR:
    gMotion->stop();
    return -1;
  }


  // Pass ball to ally
  void LSAllyPass::init() {
    gMotion->stop();
    commTimeout.setPeriod(5000);
    finish = false;
  }

  int LSAllyPass::run() {
    if(finish) return 0;

    if(mb->getBallSensorState()) {
      mb->startDribbler();
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
              gMotion->stop();

        MainBoard* mb = hal::hw.getMainBoard();
        mb->stopDribbler();
        mb->releaseCoil(); //TODO: Weak kick

        emit("done");
      }
    }
    return 0;
  }

  bool LSAllyPass::isRunnable() {
    if(finish) return true;

    if(gNavi->getAlly() == nullptr) return false;
    if(!mb->getBallSensorState()) return false;
    return true;
  }


  // Receive ball from ally
  void LSAllyReceive::init() {
    gMotion->stop();
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

}}
