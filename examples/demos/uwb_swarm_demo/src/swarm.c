/**
 * swarm.c - App layer application of the onboard swarm demo with only UWB tags.
 * The crazyflie has to have the locodeck and the flowdeck version 2.
 * 
 * Project page: https://shushuai3.github.io/swarm.html
 * Paper: https://arxiv.org/abs/2003.05853
 */

#include "app.h"
#include "commander.h"
#include "FreeRTOS.h"
#include "task.h"
#include "debug.h"
#include "log.h"
#include "param.h"

#include <stdlib.h>
#include <math.h>
#include "num.h"
#include "configblock.h"
#include "estimator_kalman.h"
#include "relative_localization.h"
#include "swarmTwrTag.h"

// creates swarm.c wide variable of stabilizer_type.c struct setpoint
static setpoint_t setpoint;
// 2D-float array with all UWB Nodes in one and X, Y, YAW in the other direction
static float rlVarForCtrl[NUM_UWB][STATE_DIM_rl];
// Id of this dorne, based on the 40 bit radio adress, but only the last hex character(the last 4 bits) are used.
static uint8_t myId;
// fixed height for all operations to 0.4 m
static float height = 0.4f;
// variable that has to be true, otherwise a landing will be initiated within the next taskloop
static bool keepFlying = false;

/* Gives a new setpoint to the High Level Controller
 *
 * Parameters:
 * - setpoint_t pointer
 * - x Velocity as float
 * - y velocity as float
 * - height as float
 * - yawrate as float
 * Returns:
 * - Nothing
 *                                                                            */
static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate) {
  // control loop around height
  setpoint->mode.z = modeAbs;
  // sets z as absolute height
  setpoint->position.z = z;
  // control loop around velocity
  setpoint->mode.yaw = modeVelocity;
  // sets yawrate as attittude
  setpoint->attitudeRate.yaw = yawrate;
  // control loop around velocity for x and y
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  // sets speed in x and y direction
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  // use airframe coordinate system instead of global coordinate system
  setpoint->velocity_body = true;
  // give the global swarm.c struct "setpoint" by value to the High Level Commander
  commanderSetSetpoint(setpoint, 3);
}

/* Makes the CF fly 1m in a random XY direction and return 1m in the counter
 * direction back to the starting point.
 *
 * Parameters:
 * - velocity of maneuver as float
 * Returns:
 * - Nothing
 *                                                                            */
static void flyRandomIn1meter(float vel) {
  // velocity direction [0 - 2pi] rad
  float randomYaw = (rand() / (float) RAND_MAX) * 6.28f;
  // velocity magnitude [0 - 1] m
  float randomVel = vel*(rand() / (float)RAND_MAX);
  // set x velocity = front and back velocity to x component of velocity vector
  float vxBody = randomVel * cosf(randomYaw);
  // set y velocity = right and left velocity to y component of velocity vector
  float vyBody = randomVel * sinf(randomYaw);
  //do the fly away maneuver for 100 * 10ms = 1s
  for (int i = 1; i < 100; i++) {
    // change the setpoint and send it to the High Level Commander
    setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
  // do the fly back maneuver respectively
  for (int i = 1; i < 100; i++) {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
}

// PID control of swarm flight
// macro SIGN returns 1 if x is positive, -1 if x is negative
#define SIGN(a) ((a >= 0) ? 1 : -1)
// sets the pid values 
static float rlPIDp = 2.0f, rlPIDi = 0.0001f, rlPIDd = 0.01f;
// creates static variables to store the Error Values of the last loop to calculate the derivative
static float PreErrX = 0, PreErrY = 0, IntErrX = 0, IntErrY = 0;
// creates static variable to store the timestamp of the last loop to calculate the derivative
static uint32_t rlPIDlastTime;

/* Uses a simple PID control on X and Y to move to specified location relative
 * to leader.
 *
 * Parameters:
 * - desired x position relative to leader as float
 * - desired y position relative to leader as float
 * Returns:
 * - Nothing
 *                                                                            */
static void moveWithLeaderAsOrigin(float posX, float posY) {
  // calculate the time difference between the last loop and this loop
  float dt = (float)(xTaskGetTickCount() - rlPIDlastTime) / configTICK_RATE_HZ;
  // remember the timestamp of this loop
  rlPIDlastTime = xTaskGetTickCount();
  // sanity check to just drop the calculation for one loop if counter overflow occured
  if (dt > 1)
    return;
  // calculate setpoint error of the position
  float errX = -(posX - rlVarForCtrl[0][STATE_rlX]);
  float errY = -(posY - rlVarForCtrl[0][STATE_rlY]);
  // calculate the proportional part for the velocity
  float pid_vx = rlPIDp * errX;
  float pid_vy = rlPIDp * errY;
  // calculate the derivative part
  float dx = (errX - PreErrX) / dt;
  float dy = (errY - PreErrY) / dt;
  // store current error for derivative in the next loop
  PreErrX = errX;
  PreErrY = errY;
  // calculate the derivative part and add it to the velocity
  pid_vx += rlPIDd * dx;
  pid_vy += rlPIDd * dy;
  // add this loops error to form the integral part
  IntErrX += errX * dt;
  IntErrY += errY * dt;
  // calculate the integral part by the constrained integral memory and add it to the velocity
  pid_vx += rlPIDi * constrain(IntErrX, -0.5, 0.5);
  pid_vy += rlPIDi * constrain(IntErrY, -0.5, 0.5);
  // constrain the calculated velocity to safe maximum value
  pid_vx = constrain(pid_vx, -1.5f, 1.5f);
  pid_vy = constrain(pid_vy, -1.5f, 1.5f);
  // change the setpoint and send it to the High Level Commander
  setHoverSetpoint(&setpoint, pid_vx, pid_vy, height, 0);
}

/* appMain function = starting point for applayer superloop
 *
 * Parameters:
 * - Nothing
 * Returns:
 * - Nothing
 *                                                                            */
void appMain() {
  vTaskDelay(M2T(3000));
  DEBUG_PRINT("Waiting for activation ...\n");

  // variable to keep track of takeoff and landing
  static bool onGround = true;
  // remember the takeoff timestamp
  static uint32_t timeTakeOff;
  // setpoint variables for xy position PID controller, gets initialized with current position
  static float desireX;
  static float desireY;
  // information for keepFlying is retrieved from the kalam filter
  static logVarId_t logIdStateIsFlying;
  logIdStateIsFlying = logGetVarId("kalman", "inFlight");
  // Id of this dorne, based on the 40 bit radio adress, but only the last hex character(the last 4 bits) are used.
  myId = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f));

  while(1) {
    vTaskDelay(M2T(10));

// if MANUAL_CONTROL_LEADER flag is set, and the last hex character is 0, the leader is actively controlled via cfclient software
#ifdef MANUAL_CONTROL_LEADER
    if (myId == 0) {
      // retrieve flight state from kalam filter
      keepFlying = logGetUint(logIdStateIsFlying);
      // update keepFlying and make it known to the swarm
      keepFlying = updateFlyStatus(myId, keepFlying);
      //run the relative localization algorithm, to get the relative position of the swarm accessable in swarm.c via rlVarForCtrl
      relative_localization((float *)rlVarForCtrl);
      continue;
    }
#endif

    keepFlying = updateFlyStatus(myId, keepFlying);

    if(relative_localization((float *)rlVarForCtrl) && keepFlying) {
      // take off
      if (onGround) {
        estimatorKalmanInit();
        vTaskDelay(M2T(2000));
        for (int i = 0; i < 50; i++) {
          setHoverSetpoint(&setpoint, 0, 0, height, 0);
          vTaskDelay(M2T(100));
        }
        onGround = false;
        timeTakeOff = xTaskGetTickCount();
      }
      uint32_t timeInAir = xTaskGetTickCount() - timeTakeOff;
      
      // 0-20s random flight
      if (timeInAir < 20000) {
        flyRandomIn1meter(1.0f);
        desireX = rlVarForCtrl[0][STATE_rlX];
        desireY = rlVarForCtrl[0][STATE_rlY];
      }

      // 20-30s formation flight
      if ((timeInAir >= 20000) && (timeInAir < 30000)) {
        moveWithLeaderAsOrigin(desireX, desireY);
      }

      // after 30s, atomic pattern flight
      if (timeInAir >= 30000) {
          float radius = (float)myId * 0.5f;
          float timeInSecond = (float)timeInAir / configTICK_RATE_HZ;
          float rlPosXofMeIn0 = radius * cosf(timeInSecond);
          float rlPosYofMeIn0 = radius * sinf(timeInSecond);
          desireX = -cosf(rlVarForCtrl[0][STATE_rlYaw]) * rlPosXofMeIn0 + sinf(rlVarForCtrl[0][STATE_rlYaw]) * rlPosYofMeIn0;
          desireY = -sinf(rlVarForCtrl[0][STATE_rlYaw]) * rlPosXofMeIn0 - cosf(rlVarForCtrl[0][STATE_rlYaw]) * rlPosYofMeIn0;
          moveWithLeaderAsOrigin(desireX, desireY);
      }

    } else {
      // landing procedure
      if(!onGround) {
        for (int i = 1; i < 5; i++) {
          setHoverSetpoint(&setpoint, 0, 0, 0.3f - (float)i * 0.05f, 0);
          vTaskDelay(M2T(10));
        }
      }
      onGround = true;
    } 
  }
}

PARAM_GROUP_START(rl_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_ADD(PARAM_FLOAT, PID_P, &rlPIDp)
PARAM_ADD(PARAM_FLOAT, PID_I, &rlPIDi)
PARAM_ADD(PARAM_FLOAT, PID_D, &rlPIDd)
PARAM_GROUP_STOP(rl_ctrl)