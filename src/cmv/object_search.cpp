
#include "core/rtx_GameField.hpp"

#include "cmv/blob_finder.hpp"
#include "cmv/object_search.hpp"

namespace rtx {

  const char* CMV_BALL = "Ball";

  const char* CMV_GOAL_PINK = "G1_pink";
  const char* CMV_GOAL_BLUE = "G2_blue";

  enum ObjectType {
    OBJ_NULL,
    OBJ_BALL,
    OBJ_GOAL,
  } objTypeBuf = OBJ_NULL;

  cmv::CMVisionColorBlobFinder blobHunter;

  bool cmv_init_flag = false;

  BlobSet balls, goals;

  int cmv_init()
  {
    if(cmv_init_flag) return 0;

    blobHunter.initialize();

    cmv_init_flag = true;
    return 0;
  }

  int object_detection_init()
  {
    cmv_init();
    return 0;
  }

  int ball_detection(cv::Mat& img, GameField* gmField)
  {
    return 0;
  }

  int goal_detection(cv::Mat& img, GameField* gmField)
  {
    return 0;
  }

  int object_detection(cv::Mat& img, GameField* gmField)
  {
    auto blob_cb = [](cmv::blob_t *ptr) {
      std::string n = ptr->name;

      if(n == CMV_BALL) objTypeBuf = OBJ_BALL;
      else if(n == CMV_GOAL_PINK) objTypeBuf = OBJ_GOAL;
      else if(n == CMV_GOAL_BLUE) objTypeBuf = OBJ_GOAL;

      switch(objTypeBuf) {
        case OBJ_BALL:
          balls.push_back(*ptr);
          break;
        case OBJ_GOAL:
          goals.push_back(*ptr);
          break;
      }
    };

    int res = blobHunter.findAllBlobs(img, blob_cb);
    //printf("balls_n=%u, goals_n=%u\n", balls.size(), goals.size());

    if(balls.size() > 0)
    {
      gmField->digestBallBlobs(balls);
      balls.clear();
    }

    if(goals.size() > 0)
    {
      gmField->digestGoalBlobs(goals);
      goals.clear();
    }

    return 0;
  }

}
