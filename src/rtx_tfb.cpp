/** @file tfb_logic.cpp
 *  Team football logic module implementation.
 *
 *  @authors Meelik Kiik
 *  @version 0.1
 *  @date 2. November 2015
 */

#include "RefereeListener.hpp"
#include "STM.hpp"
#include "LogicManager.hpp"

#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "rtx_tfb.hpp"

using namespace tuum::hal;

namespace tuum { namespace TFBLogic {

  GameState gmState;
  GamePhase gmPhase;

  // Preload logic trees
  STM* lg_preKickoff = nullptr;
  STM* lg_kickoff = nullptr;

  STM* lg_defKickoff = nullptr;


  STM* logicProcess;


  void setup() {
    stop();
    init_referee_signals();
    load_logic();

    updateGameState(GameState::STOP);
    updateGamePhase(GamePhase::KICKOFF, true);
  }

  void process() {
    switch(gmState) {
      case GameState::STOP:
        if(gMotion->isRunning()) gMotion->stop();
        break;
      case GameState::PLACEDBALL:
        //TODO: prepare for game phase?
        break;
      case GameState::START:
        if(logicProcess != nullptr) logicProcess->process();
        break;
    }
  }


  // Callbacks
  void start() {
    std::cout << "Start game." << std::endl;
    gmState = GameState::START;
  }

  void stop() {
    std::cout << "Stop game." << std::endl;
    gmState = GameState::STOP;
  }


  // Initialization methods
  void init_referee_signals() {
    RefereeListener* ref = hw.getRefListener();

    ref->registerCallback(REF_START, [=](RefCommand rcmd){
      start();
    });

    ref->registerCallback(REF_STOP, [=](RefCommand rcmd){
      stop();
      gMotion->stop();
      hw.getMotorControl()->omniDrive(0, 0.0, 0);
      MainBoard* mb = hw.getMainBoard();
      mb->stopDribbler();
    });

    ref->registerCallback(REF_KICKOFF, [=](RefCommand rcmd){
      RefereeListener* r = hw.getRefListener();
      updateGamePhase(GamePhase::KICKOFF, rcmd.target.team == r->m_team);
    });
  }

  void load_logic() {
    std::string v;

    v = gC.getStr("Robot.Role");
    if(v == "Attacker") {
      lg_preKickoff = LogicManager::loadKickoffReceiverPrepare();
      lg_kickoff = LogicManager::loadKickoffReceiver();
    } else if(v == "Goalee") {
      lg_preKickoff = LogicManager::loadKickoffPasserPrepare();
      lg_kickoff = LogicManager::loadKickoffPasser();
     }
  }


  // State update methods
  void updateGamePhase(GamePhase gp, bool to_our_team) {

    std::string role;
    gmPhase = gp;
    role = gC.getStr("Robot.Role");

    switch(gmPhase) {
      case GamePhase::KICKOFF:
        std::cout << "Transition to ";
        if(to_our_team) {
          std::cout << "our";
          logicProcess = lg_kickoff;
          logicProcess->setup();

        } else {
          std::cout << "opposing";
          //TODO
        }

        logicProcess->registerEventListener(logicProcess->getEventID("LSDone"), [=](){
          updateGamePhase(GamePhase::GAME);
        });

        break;
      case GamePhase::GAME:
        if(role == "Attacker")
        {
          logicProcess = LogicManager::loadOffensivePlay();
        }
        else if(role == "Goalee")
        {
          logicProcess = LogicManager::loadDefensivePlay();
        }
        logicProcess->setup();
        break;
      case GamePhase::KICKOFF_GOAL:
        break;
      case GamePhase::FREEKICK_DIRECT:
        if(role == "Attacker")
        {
          logicProcess = LogicManager::moveToBall();
          logicProcess->setup();
        }
        break;
      case GamePhase::FREEKICK_INDIRECT:
        if(role == "Goalee")
        {
          logicProcess = LogicManager::moveToBall();
        }
        else
        {
            logicProcess = lg_preKickoff;
        }
        logicProcess->setup();
        break;
      case GamePhase::THROWIN:
      if(role == "Goalee")
      {
        logicProcess = LogicManager::moveToBall();
      }
      else
      {
          logicProcess = lg_preKickoff;
      }
      logicProcess->setup();
        break;
      case GamePhase::NONE:
      default:
        logicProcess = nullptr;
        break;
    }
  }

  void updateGameState(GameState gs) {
    gmState = gs;
  }

}}
