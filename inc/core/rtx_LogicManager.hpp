/** @file LogicManager.hpp
 *  Logic machines loading class.
 *
 *  @authors Meelik Kiik
 *  @version 0.1
 *  @date 28. November 2015
 */

#ifndef RTX_LOGIC_MANAGER_H
#define RTX_LOGIC_MANAGER_H

#include "STM.hpp"

using namespace tuum;

namespace rtx {

  class LogicManager {
  public:
    static STM* loadKickoffReceiverPrepare();
    static STM* loadKickoffReceiver();

    static STM* loadKickoffPasserPrepare();
    static STM* loadKickoffPasser();

    static STM* loadOffensivePlay();
    static STM* loadDefensivePlay();

    static STM* placedBall();
    static STM* enemyKikcoff();
    static STM* enemyKikcoffPrepare();
    static STM* throwinPrepare();

    static STM* stmPlaceholder();
  };

}

#endif // RTX_LOGIC_MANAGER_H
