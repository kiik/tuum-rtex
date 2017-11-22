
#include "cmv/object_search.hpp"

#include "core/rtx_GameField.hpp"

namespace rtx {

  GameField::GameField():
    mGoalPos(0, 0),
    mGoalConfidence(0.0),

    m_goal_buf_N(10),
    m_goal_buf(new Vec2i[m_goal_buf_N]),
    m_goal_buf_ix(0)
  {

  }

  void GameField::tick()
  {
    updateBalls();
    updateGoals();

    mG1_pink.tick();
    mG1_pink.tick();
  }

  void GameField::updateBalls()
  {
    BallHandle ptr;
    for(auto it = mBalls.begin(); it != mBalls.end(); ++it) {
      ptr = *it;

      if(ptr->deadFrameCount() >= 5)
      {
        auto p = ptr->getBlob()->getCentroid();
        printf("LOST Ball#%lu(%i, %i)\n", ptr->getId(), p.x, p.y);
        mBalls.erase(it);
      }
    }
  }

  void GameField::updateGoals()
  {
    GoalHandle ptr;
    for(auto it = mGoals.begin(); it != mGoals.end(); ++it) {
      ptr = *it;

      if(ptr->deadFrameCount() >= 5)
      {
        auto p = ptr->getBlob()->getCentroid();
        printf("LOST Goal#%lu(%i, %i)\n", ptr->getId(), p.x, p.y);
        mGoals.erase(it);
      }
    }
  }

  void GameField::digestBallBlob(cmv::blob_t& bl)
  {
    // printf("[GameField::digestBallBlob]GOT: %s\n", bl.name.c_str());

    //TODO: Apply undistortion, T_cameraToWorld
    Blob blob(bl.name, {
      .rect = {bl.left, bl.top, bl.right, bl.bottom},
      .realArea = bl.area
    });

    for(auto &ball : mBalls)
    {
      // if(ball.matched()) continue;

      // Match blob to entity from last frame
      if(ball->matchPercent(blob) > 0.5)
      {
        ball->match(blob);
        return;
      }
    }

    BallHandle nBallHandle(new Ball((const Blob&)blob));

    printf("[GameField::digestBallBlob]NEW Ball#%lu(%i, %i)\n", nBallHandle->getId(), bl.c_x, bl.c_y);
    mBalls.push_back(nBallHandle);
  }

  void GameField::digestGoalBlob(cmv::blob_t& bl)
  {
    // printf("[GameField::digestGoalBlob]GOT: %s\n", bl.name.c_str());

    Blob blob(bl.name, {
      .rect = {bl.left, bl.top, bl.right, bl.bottom},
      .realArea = bl.area
    });

    const float matchCondition = 0.5;

    for(auto &goal : mGoals)
    {
      // Match blob to entity from last frame
      if(goal->matchPercent(blob) > matchCondition)
      {
        goal->match(blob);
        return;
      }
    }

    GoalHandle nGoalHandle(new Goal((const Blob&)blob));

    printf("[GameField::digestGoalBlob]NEW Goal#%lu(%i, %i)\n", nGoalHandle->getId(), bl.c_x, bl.c_y);
    mGoals.push_back(nGoalHandle);
  }

  void GameField::digestBallBlobs(BlobSet& bls)
  {
    for(auto &bl : bls) digestBallBlob(bl);
  }

  void GameField::digestGoalBlobs(BlobSet& bls)
  {
    for(auto &bl : bls) digestGoalBlob(bl);
  }

  void GameField::digestGoalMarkers(MarkerSet& markers, int W_2, int H)
  {
    const unsigned char ally_id_left = 20, ally_id_right = 21;
    const unsigned char oppn_id_left = 10, oppn_id_right = 11;
    const float Z_critical = 200.0;

    aruco::Marker *ally_ptr_l = nullptr, *ally_ptr_r = nullptr,
                  *op_ptr_l = nullptr, *op_ptr_r = nullptr;

    for(auto &marker : markers)
    {
      switch(marker.id) {
        case oppn_id_left:
          op_ptr_l = &marker;
          break;
        case oppn_id_right:
          op_ptr_r = &marker;
          break;
        case ally_id_left:
          ally_ptr_l = &marker;
          break;
        case ally_id_right:
          ally_ptr_r = &marker;
          break;
      }
    }

    const cv::Point2f Zvec(0, 0);

    // Translation, pixel translation vectors.
    cv::Point2f Tvec(0, 0), px_Tvec(0, 0);

    if( (op_ptr_l != nullptr) || (op_ptr_r != nullptr) )
    {
      cv::Point2f c_l, c_r;

      if(op_ptr_l == nullptr)
      {
        c_r = op_ptr_r->getCenter();
      }
      else if(op_ptr_r == nullptr)
      {
        c_l = op_ptr_l->getCenter();
      }
      else
      {
        c_l = op_ptr_l->getCenter();
        c_r = op_ptr_r->getCenter();

        px_Tvec.x = W_2 - (c_l.x + c_r.x) / 2.0;
        px_Tvec.y = H - (c_l.y + c_r.y) / 2.0;

        m_goal_buf[m_goal_buf_ix].x = px_Tvec.x;
        m_goal_buf[m_goal_buf_ix].x = px_Tvec.y;

        m_goal_buf_ix = (m_goal_buf_ix + 1) % m_goal_buf_N;

        m_goal_avg = m_goal_buf[0];

        for(int i = 1; i < m_goal_buf_N; ++i)
        {
          m_goal_avg.x += m_goal_buf[i].x;
          m_goal_avg.y += m_goal_buf[i].y;
        }

        m_goal_avg.x /= m_goal_buf_N;
        m_goal_avg.y /= m_goal_buf_N;

        m_goal_stddev = Vec2f(0, 0);

        for(int i = 1; i < m_goal_buf_N; ++i)
        {
          m_goal_stddev.x += pow(m_goal_avg.x - m_goal_buf[i].x, 2);
          m_goal_stddev.y += pow(m_goal_avg.y - m_goal_buf[i].y, 2);
        }

        m_goal_stddev.x /= m_goal_buf_N;
        m_goal_stddev.y /= m_goal_buf_N;

        float mn, mx, dx;
        float span = Z_critical * m_goal_stddev.x / pow(m_goal_buf_N - 1, 0.5);

        mn = m_goal_avg.x - span, mx = m_goal_avg.y + span;

        // m_goal_avg.x = 100%, m_goal_avg.x - span  - 0%
        // px_TVec.x  = P

        if(m_goal_stddev.x < 1.0) mGoalConfidence = 1.0;
        else
        {
          dx = abs(m_goal_avg.x - px_Tvec.x);
          printf("dx=%.2f\n", dx);
          if(px_Tvec.x < span) mGoalConfidence = 0;
          else
          {
            mGoalConfidence = dx / span;
          }
        }

        //mGoal.updatePosition()

        //cout << (*op_ptr_l) << endl;
        //cout << (*op_ptr_r) << endl;

        /*
        cout << px_Tvec
          << ", stddev=(" << m_goal_stddev.x << ", " << m_goal_stddev.y
          << "), p=" << mGoalConfidence
          << ", span=" << span
          << endl;
          */
      }
    }

  }

  Entity* GameField::getAlly()
  {
    return nullptr;
  }

  BallHandle GameField::getNearestBall()
  {
    BallHandle ptr = nullptr;
    float d0, d;

    for(auto &bl : mBalls)
    {
      if(ptr == nullptr)
      {
        ptr = bl;
        d0  = ptr->getTransform()->getPosition().getMagnitude();
        continue;
      }

      d = bl->getTransform()->getPosition().getMagnitude();

      if(d < d0) {
        ptr = bl;
        d0 = d;
      }
    }

    return ptr;
  }

  GoalHandle GameField::getAllyGoal()
  {
    return nullptr;
  }

  GoalHandle GameField::getOpponentGoal()
  {
    //if(mGoalConfidence < 0.5) return nullptr;
    for(auto &gl : mGoals)
    {
      return gl;
    }

    return nullptr;
  }

  unsigned int GameField::countValidBalls()
  {
    return mBalls.size();
  }

  Transform GameField::calcBallPickupPos(Transform* in)
  {
    Transform out = *in;
    return out;
  }

  Transform GameField::ballPickupPos(BallHandle bl, GoalHandle gl)
  {
    Transform out;

    auto p0 = bl->getTransform()->getPosition(),
         p1 = gl->getTransform()->getPosition();

    Vec2D<double> avec = (p1 - p0).getNormalized();
    Vec2i pvec = p0 - avec * 55;

    out.setPosition(pvec);
    out.setOrientation(avec.getOrientation());

    return out;
  }

  Transform GameField::calcAllyGoalPos(Transform* in)
  {
    Transform out = *in;
    return out;
  }

}
