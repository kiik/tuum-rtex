
#ifndef RTX_GAMEFIELD_H
#define RTX_GAMEFIELD_H

#include <opencv2/opencv.hpp>

#include "Vec.hpp"

#include "Transform.hpp"
#include "Entity.hpp"
#include "Ball.hpp"
#include "Goal.hpp"

#include "cmv/object_search.hpp"
#include "cmv/marker_search.hpp"

using namespace tuum;

namespace rtx {

  typedef Ball* BallHandle;
  typedef std::vector<BallHandle> BallSet;

  typedef Goal* GoalHandle;
  typedef std::vector<GoalHandle> GoalSet;

  class GameField
  {
  public:
    GameField();

    void tick();

    void updateBalls();
    void updateGoals();

    void digestBallBlob(cmv::blob_t&);
    void digestGoalBlob(cmv::blob_t&);

    void digestBallBlobs(BlobSet&);
    void digestGoalBlobs(BlobSet&);

    void digestGoalMarker(aruco::Marker&);
    void digestGoalMarkers(MarkerSet&, int, int);

    BallSet* getBallsHandle() { return &mBalls; }
    GoalSet* getGoalsHandle() { return &mGoals; }

    Entity* getAlly();

    BallHandle getNearestBall();

    GoalHandle getAllyGoal();
    GoalHandle getOpponentGoal();

    unsigned int countValidBalls();

    Transform calcBallPickupPos(Transform*);
    Transform calcAllyGoalPos(Transform*);

    Transform ballPickupPos(BallHandle, GoalHandle);

    void setCameraParams(cv::Mat camMx, cv::Mat distCoeff);

  private:
    BallSet mBalls;
    GoalSet mGoals;

    // Goal mG1_pink, mG2_blue;

    Vec2i mGoalPos;
    float mGoalConfidence;

    unsigned int m_goal_buf_N, m_goal_buf_ix;
    Vec2i *m_goal_buf, m_goal_avg, m_goal_stddev;

    cv::Mat mCamMx, mDistCoeff;
  };

}

#endif
