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

#include "hal.hpp"

#include "rtx_ctl.hpp"

using namespace tuum::comm;
using namespace tuum::hal;

namespace tuum {

  //TODO: Refactor into separate module.
  namespace Motion {

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
          Motion::setAimTarget(Vec2i({0, 1}));
          motionData.manualRotGear = {m_spv, 3.14};
          Motion::start();
          wait_for_vision = true;
          visionTimer.start();
        }
      } else {
        if(visionTimer.isTime()) {
          Motion::setAimTarget(Vec2i({0, 1}));
          motionData.manualRotGear = {m_sps, 3.14};
          Motion::start();
          wait_for_vision = false;
          motionTimer.start();
        }
      }
    }

  }

}

namespace tuum { namespace ctl {

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
    switch(ctx.phase) {
      case CP_INIT:
        Motion::setBehaviour(Motion::MOT_CURVED);
        Motion::setTarget(Transform(10, 10, 0));

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
    Motion::stop();
    Motion::setBehaviour(Motion::MOT_COMPLEX);
    twitchScanner.init(5, 30);
    mb->stopDribbler();
  }

  int LSBallLocate::run() {
    if(gNavigation->countValidBalls() > 0) {
      mb->startDribbler();
      Motion::stop();
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
    Motion::stop();

    Ball* b = gNavigation->getNearestBall();
    if(b != nullptr)
      std::cout << "Navigate to " << b->toString() << std::endl;
  }

  int LSBallNavigator::run() {
    Ball* b = nullptr;
    if(mb->getBallSensorState()) goto OK;
    if(gNavigation->countValidBalls() <= 0) goto ERR;

    b = gNavigation->getNearestBall();

    if(b != nullptr) {
      Vec2i pos = gNavigation->calcBallPickupPos(b->getTransform()).getPosition();

      Motion::setPositionTarget(pos);
      Motion::setAimTarget(b->getTransform()->getPosition());
      mb->startDribbler();

      //std::cout << d << std::endl;
      //if(d < 250) mb->startDribbler();
      //else mb->stopDribbler();

      if(!Motion::isTargetAchieved()) {
        if(!Motion::isRunning()) Motion::start();
      } else {
        goto OK;
      }
    } else {
      Motion::stop();
    }

    return 0;
OK:
    Motion::stop();
    return 1;
ERR:
    Motion::stop();
    return -1;
  }

  bool LSBallNavigator::isRunnable() {
    return gNavigation->countValidBalls() > 0 || mb->getBallSensorState();
  }


  // Ball pickup
  void LSBallPicker::init() {
    Motion::stop();
    mb->startDribbler();
  }

  int LSBallPicker::run() {
    Ball* b = nullptr;
    if(mb->getBallSensorState()) goto OK;
    if(gNavigation->countValidBalls() <= 0) goto ERR;

    b = gNavigation->getNearestBall();

    if(b != nullptr) {
      double dD = Motion::VLS_DIST.low;
      Transform* t = b->getTransform();
      Transform* me = Localization::getTransform();
      Vec2f avf = (t->getPosition() - me->getPosition()).getNormalized();
      Motion::setPositionTarget(t->getPosition() + (avf*dD).toInt());
      Motion::setAimTarget(t->getPosition() + (avf*1.1*dD).toInt());

      if(me->getPosition().distanceTo(t->getPosition()) > dD) return -1;

      mb->startDribbler();
      if(!Motion::isRunning()) Motion::start();
    } else {
      Motion::stop();
    }

    return 0;
OK:
    Motion::stop();
    return 1;
ERR:
    Motion::stop();
    mb->stopDribbler();
    return -1;
  }

  bool LSBallPicker::isRunnable() {
    if(mb->getBallSensorState()) return true;

    Ball* b = gNavigation->getNearestBall();
    if(b == nullptr) return false;

    Transform* t = Localization::getTransform();
    double d = t->distanceTo(b->getTransform()->getPosition());
    if(d > (Motion::VLS_DIST.mn + Motion::GRS_MOV.mn.step) ) return false;

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
    Motion::stop();
    ctx.phase = CP_INIT;
    twitchScanner.init(10, 30);
    mb->startDribbler();
  }

  int LSGoalLocate::run() {
    if(!mb->getBallSensorState()) goto ERR;
    if(gNavigation->getOpponentGoal() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    Motion::stop();
    return 1;
ERR:
    Motion::stop();
    return -1;
  }


  ////////////////////
  void LSAllyGoalLocate::init() {
    Motion::stop();
    ctx.phase = CP_INIT;
    twitchScanner.init(10, 30);
  }

  int LSAllyGoalLocate::run() {
    if(gNavigation->getAllyGoal() != nullptr) return 1;

    twitchScanner.run();

    return 0;
  }

  void LSAllyGoalMove::init() {
    Motion::stop();

    Goal* goal = gNavigation->getAllyGoal();
    if(goal != nullptr)
      std::cout << "Navigate to " << goal->toString() << std::endl;
  }

  int LSAllyGoalMove::run() {
    Goal* goal = nullptr;
    //if(värava ees) goto OK;
    //if(väravat ei leitud) goto ERR;

    if(goal != nullptr) {
      Vec2i pos = gNavigation->calcAllyGoalPos(goal->getTransform()).getPosition();

      Motion::setPositionTarget(pos);
      Motion::setAimTarget(goal->getTransform()->getPosition());

      //std::cout << d << std::endl;
      //if(d < 250) mb->startDribbler();
      //else mb->stopDribbler();

      if(!Motion::isTargetAchieved()) {
        if(!Motion::isRunning()) Motion::start();
      } else {
        goto OK;
      }
    } else {
      Motion::stop();
    }

    return 0;
OK:
    Motion::stop();
    return 1;
ERR:
    Motion::stop();
    return -1;
  }

  bool LSAllyGoalMove::isRunnable() {
    return true;
  }
  ////////////////////////////


  // Shoot to opposing goal
  void LSGoalShoot::init() {
    Motion::stop();
  }

  int LSGoalShoot::run() {
    if(!mb->getBallSensorState()) return -1;

    Goal* g = gNavigation->getOpponentGoal();
    if(g == nullptr) return -1;

    //Motion::setPositionTarget(gNavigation->getGoalShootPosition(g));
    Motion::setAimTarget(g->getTransform()->getPosition());
    //std::cout << g->getTransform()->getPosition().toString() << std::endl;;
;
    //if(fabs(Motion::getDeltaOrientation()) < 0.030) mb->releaseCoil();

    if(!Motion::isTargetAchieved()) {
      if(!Motion::isRunning()) Motion::start();
    } else {
      Motion::stop();

      if(mb->getBallSensorState()) mb->releaseCoil();

    }

    return 0;
  }

  bool LSGoalShoot::isRunnable() {
    return gNavigation->getOpponentGoal() != nullptr && mb->getBallSensorState();
  }

  // Defend goal
  void LSGoalee::init() {
    Motion::stop();
    Motion::setBehaviour(Motion::MOT_COMPLEX);
  }

  int LSGoalee::run() {
    Ball* b = gNavigation->getNearestBall();

    if(b != nullptr) {
      Vec2i pos = gNavigation->calcBallPickupPos(b->getTransform()).getPosition();

      Motion::setPositionTarget(pos);
      Motion::setAimTarget(b->getTransform()->getPosition());

      Transform* t = Localization::getTransform();
      double d = t->distanceTo(b->getTransform()->getPosition());

      //std::cout << d << std::endl;
      //if(d < 250) mb->startDribbler();
      //else mb->stopDribbler();

      if(!Motion::isTargetAchieved()) {
        if(!Motion::isRunning()) Motion::start();
      }
    } else {
      Motion::stop();
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
    Motion::stop();
    twitchScanner.init(10, 30);
  }

  int LSAllyFind::run() {
    if(gNavigation->getAlly() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    Motion::stop();
    return 1;
ERR:
    Motion::stop();
    return -1;
  }


  bool LSAllyLocate::isRunnable() {
    if(mb->getBallSensorState()) return true;
    return false;
  }

  void LSAllyLocate::init() {
    Motion::stop();
    twitchScanner.init(10, 30);
  }

  int LSAllyLocate::run() {
    if(gNavigation->getAlly() != nullptr) goto OK;

    twitchScanner.run();

    return 0;
OK:
    Motion::stop();
    return 1;
ERR:
    Motion::stop();
    return -1;
  }


  bool LSAllyAim::isRunnable() {
    return gNavigation->getAlly() != nullptr;
  }

  void LSAllyAim::init() {
    Motion::stop();
  }

  int LSAllyAim::run() {
    if(gNavigation->getAlly() == nullptr) goto ERR;

    Motion::setAimTarget(gNavigation->getAlly()->getTransform()->getPosition());
    if(!Motion::isRunning()) Motion::start();

    return 0;
OK:
    Motion::stop();
    return 1;
ERR:
    Motion::stop();
    return -1;
  }


  // Pass ball to ally
  void LSAllyPass::init() {
    Motion::stop();
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
              Motion::stop();

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

    if(gNavigation->getAlly() == nullptr) return false;
    if(!mb->getBallSensorState()) return false;
    return true;
  }


  // Receive ball from ally
  void LSAllyReceive::init() {
    Motion::stop();
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
    Motion::stop();
  }

  int LSPlacedBallInit::run() {
    //Move to ball until 500mm away

    /*Ball* b = gNavigation->getNearestBall();

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

  void LSWaitForEnemyKickoff::init(){
    Motion::stop();
    kickoffTimer.setPeriod(5000);
  }

  int LSWaitForEnemyKickoff::run(){
    kickoffTimer.start();

    while(!gameStarted){

      if (kickoffTimer.isTime()) {
        gameStarted = true;
      }
      /*else if(the ball has moved more than 10cm){
        gameStarted = true;
      }*/
    }

  }

  //void scanForBallMovement

  void LSMoveToEntity::init() {
    //Scan for entity
  }

  int LSMoveToEntity::run(){

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

}}
