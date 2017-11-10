
#ifndef RTX_GAMEFIELD_H
#define RTX_GAMEFIELD_H

#include "Vec.hpp"

#include "Transform.hpp"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Goal.hpp"

namespace tuum {

  class GameField
  {
  public:
    GameField();

    Entity* getAlly();

    Ball* getNearestBall();

    Goal* getAllyGoal();
    Goal* getOpponentGoal();

    unsigned int countValidBalls();

    Transform calcBallPickupPos(Transform*);
    Transform calcAllyGoalPos(Transform*);

  private:
  };

}

#endif
