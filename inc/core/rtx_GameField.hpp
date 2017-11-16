
#ifndef RTX_GAMEFIELD_H
#define RTX_GAMEFIELD_H

#include <opencv2/opencv.hpp>

#include "Vec.hpp"

#include "Transform.hpp"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Goal.hpp"

#include "rtx_goal_detect.hpp"

using namespace tuum;

namespace rtx {

  class GameField
  {
  public:
    GameField();

    void tick();

    void digestGoalMarkers(MarkerSet&, int, int);

    Entity* getAlly();

    Ball* getNearestBall();

    Goal* getAllyGoal();
    Goal* getOpponentGoal();

    unsigned int countValidBalls();

    Transform calcBallPickupPos(Transform*);
    Transform calcAllyGoalPos(Transform*);

  private:
    Goal mGoal;
    Vec2i mGoalPos;
    float mGoalConfidence;

    unsigned int m_goal_buf_N, m_goal_buf_ix;
    Vec2i *m_goal_buf, m_goal_avg, m_goal_stddev;
  };

}

#endif
