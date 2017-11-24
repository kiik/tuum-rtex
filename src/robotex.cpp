
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "tuum_visioning.hpp"
#include "tuum_localization.hpp"
#include "tuum_navigator.hpp"
#include "tuum_motion.hpp"
#include "tuum_context.hpp"

#include "pid.hpp"
#include "STM.hpp"

#include "hal.hpp"

#include "core/rtx_LogicManager.hpp"
#include "core/rtx_GameField.hpp"
#include "game/rtx_basketball.hpp"

#include "rtx_cmv.hpp"
#include "rtx_ctl.hpp"

#include "robotex.hpp"

#include "MainBoard.hpp"

using namespace tuum;

namespace misc {
  cv::Mat resize2(const cv::Mat &in, int width) {
    cv::Mat im2;

    float yf = float(width) / float(in.size().width);

    cv::resize(in, im2, cv::Size(width, float(in.size().height)*yf));

    return im2;
  }
}

namespace rtx {

  GameField *gGameField = nullptr;

  tuum::Navigator *gNav = nullptr;

  cv::Mat frameBuffer, frameBackBuffer;

  tuum::pid_t velocityControl, orientControl;

  struct robot_state_t {
    Vec2i pos = Vec2i(0,0);
    float ori = 0.0;

    Vec2f motionVector;
    float velocity = 0, angularVelocity = 0;
    time_ms_t _t0, _t1;
  } simRobot;

  void motion_handler_init()
  {
    libPID.init(&velocityControl);
    libPID.set_tuning(1.6, 1.0, 0.1, &velocityControl);
    libPID.set_limit(-100, 100, &velocityControl);
    libPID.set_period_ms(1000 / 50, &velocityControl);

    libPID.init(&orientControl);
    libPID.set_tuning(0.5, 0.0, 0.0, &orientControl);
    libPID.set_limit(-45, 45, &orientControl);
    libPID.set_period_ms(1000 / 50, &orientControl);

    velocityControl._direction = PID_REVERSE;
    orientControl._direction = PID_REVERSE;

    simRobot._t0 = millis();
  }

  void sim_tick(robot_state_t& iState)
  {
    iState._t1 = millis();

    time_ms_t dt_ms = iState._t1 - iState._t0;

    if(dt_ms < 30) return;

    double dt = dt_ms / 1000.0;

    // Do simulation step
    Vec2f mvec = iState.motionVector * (dt * iState.velocity);

    printf("mvec =(%.2f, %.2f)\n", mvec.x, mvec.y);

    iState.pos += Vec2i(mvec.x, mvec.y);
    iState.ori += dt * iState.angularVelocity;

    iState._t0 = iState._t1;
  }

  int motion_handler(Transform motionDelta)
  {
    // Velocity update step
    auto deltaPos = motionDelta.getPosition();

    double deltaDistance,
           deltaOrient   = (motionDelta.getOrientation() - (M_PI / 2.0))  * 180.0 / M_PI; // T_globalToLocal

    velocityControl.SV = 0; // Set distance target
    orientControl.SV = 0;

    // Apply simulation
    deltaDistance += (deltaPos + simRobot.pos).getMagnitude();
    deltaOrient   += simRobot.ori;

    // PID Update. Relative input -> pid sv = 0
    libPID.pid_tick(deltaDistance, &velocityControl);
    libPID.pid_tick(deltaOrient, &orientControl);

    // Retrieve calculated velocities
    int16_t velocity = velocityControl.out, angularVelocity = orientControl.out;
    float motionDirection = deltaPos.getOrientation() - (M_PI / 2.0);

    // Update simulation
    simRobot.motionVector = deltaPos.getNormalized();
    simRobot.velocity = velocity;
    simRobot.angularVelocity = angularVelocity;

    sim_tick(simRobot);

    printf("[rtx::motion_handler]input: {.deltaPos = (%i, %i), .deltaDistance = %.2f, .deltaOrient = %.2f}\n", deltaPos.x, deltaPos.y, deltaDistance, deltaOrient);

    printf("[rtx::motion_handler]t=%lu, SV/deltaDistance=%.2f, PV=%.2f, PID/gVelocity=%.2f\n",
      velocityControl._t, velocityControl.SV, velocityControl._lastProcessValue, velocityControl.out);

    //printf("[rtx::motion_handler]t=%lu, SV/deltaOrient=%.2f, PV=%.2f, PID/gTurnVelocity=%.2f\n",
    //    orientControl._t, orientControl.SV, orientControl._lastProcessValue, orientControl.out);


    //printf("[rtx::motion_handler]#TODO omniDrive(%i, %.2f, %i)\n",
    //  velocity, motionDirection, angularVelocity);

    return 0;
  }

  int init()
  {
    rtx::object_detection_init();
    rtx::marker_detection_init();

    gNav = new tuum::Navigator();
    gSys.insmod(gNav);

    motion_handler_init();
    gNav->setMotionHandler(motion_handler);

    gGameField = new GameField();
  }

  void setup()
  {
    gSys.setup();

    Basketball::setup();
  }

  void process() {
    gSys.process();

    hal::hw.readFrame(frameBackBuffer);
    frameBackBuffer = misc::resize2(frameBackBuffer, 1280);
    frameBackBuffer.copyTo(frameBuffer);

    rtx::marker_detection(frameBuffer, gGameField);
    rtx::object_detection(frameBackBuffer, gGameField);

    gGameField->tick();

    Basketball::process();

    auto ctx = gNav->getContext();

    {
      cv::Point p0(simRobot.pos.x, simRobot.pos.y);
      cv::putText(frameBuffer, "SIM", p0, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 1.4);
    }

    // rtx::cmv_ui_set_nav(ctx);
    // rtx::cmv_ui(frameBuffer, gGameField);
  }

}
