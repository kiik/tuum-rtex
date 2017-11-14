
#include "core/rtx_GameField.hpp"

namespace rtx {

  GameField::GameField()
  {

  }

  Entity* GameField::getAlly()
  {
    return nullptr;
  }

  Ball* GameField::getNearestBall()
  {
    return nullptr;
  }

  Goal* GameField::getAllyGoal()
  {
    return nullptr;
  }

  Goal* GameField::getOpponentGoal()
  {
    return nullptr;
  }

  unsigned int GameField::countValidBalls()
  {
    return 0;
  }

  Transform GameField::calcBallPickupPos(Transform* in)
  {
    Transform out = *in;
    return out;
  }

  Transform GameField::calcAllyGoalPos(Transform* in)
  {
    Transform out = *in;
    return out;
  }

}
