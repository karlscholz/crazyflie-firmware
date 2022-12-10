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

static setpoint_t setpoint;
static float rlVarForCtrl[NUM_UWB][STATE_DIM_rl];
static uint8_t myId;
static float height = 0.4f;
static float xydistance = 1.0f;
static bool keepFlying = false;

const float maxVel = 0.1f; // 1.5f; in m/s

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate) {
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

static void flyRandomIn1meter(float vel) {
  // velocity direction [0 - 2pi] rad
  float randomYaw = (rand() / (float) RAND_MAX) * 6.28f;
  // velocity magnitude [0 - 1] m
  float randomVel = vel*(rand() / (float)RAND_MAX);
  float vxBody = randomVel * cosf(randomYaw);
  float vyBody = randomVel * sinf(randomYaw);
  for (int i = 1; i < 100; i++) {
    setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
  for (int i = 1; i < 100; i++) {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
}

// PID control of swarm flight
#define SIGN(a) ((a >= 0) ? 1 : -1)
static float rlPIDp = 2.0f, rlPIDi = 0.0001f, rlPIDd = 0.01f;
static float PreErrX = 0, PreErrY = 0, IntErrX = 0, IntErrY = 0;
static uint32_t rlPIDlastTime;
static void moveWithLeaderAsOrigin(float posX, float posY) {
  float dt = (float)(xTaskGetTickCount() - rlPIDlastTime) / configTICK_RATE_HZ;
  rlPIDlastTime = xTaskGetTickCount();
  if (dt > 1)
    return;
  float errX = -(posX - rlVarForCtrl[0][STATE_rlX]);
  float errY = -(posY - rlVarForCtrl[0][STATE_rlY]);
  float pid_vx = rlPIDp * errX;
  float pid_vy = rlPIDp * errY;
  float dx = (errX - PreErrX) / dt;
  float dy = (errY - PreErrY) / dt;
  PreErrX = errX;
  PreErrY = errY;
  pid_vx += rlPIDd * dx;
  pid_vy += rlPIDd * dy;
  IntErrX += errX * dt;
  IntErrY += errY * dt;
  pid_vx += rlPIDi * constrain(IntErrX, -0.5, 0.5);
  pid_vy += rlPIDi * constrain(IntErrY, -0.5, 0.5);
  pid_vx = constrain(pid_vx, -maxVel, maxVel);
  pid_vy = constrain(pid_vy, -maxVel, maxVel);
  setHoverSetpoint(&setpoint, pid_vx, pid_vy, height, 0);
}

void appMain() {
  vTaskDelay(M2T(3000));
  DEBUG_PRINT("Waiting for activation ...\n");

  static bool onGround = true;
  static uint32_t timeTakeOff;
  static float desireX;
  static float desireY;
  static logVarId_t logIdStateIsFlying;
  logIdStateIsFlying = logGetVarId("kalman", "inFlight");
  myId = (uint8_t)(((configblockGetRadioAddress()) & 0x000000000f));

  while(1) {
    vTaskDelay(M2T(10));

#ifdef MANUAL_CONTROL_LEADER
    if (myId == 0) {
      keepFlying = logGetUint(logIdStateIsFlying);
      keepFlying = updateFlyStatus(myId, keepFlying);
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
      if (timeInAir < 30000) {
        flyRandomIn1meter(1.0f);
        desireX = rlVarForCtrl[0][STATE_rlX];
        desireY = rlVarForCtrl[0][STATE_rlY];
      }

      // // 20-30s formation flight
      // if ((timeInAir >= 20000) && (timeInAir < 30000)) {
      //   moveWithLeaderAsOrigin(desireX, desireY);
      // }

      // after 30s, diamond pattern flight
      if (timeInAir >= 30000) {
          float rlPosXofMeIn0;
          float rlPosYofMeIn0;
          switch(myId){
            case 1: // left
              rlPosXofMeIn0 = -xydistance;
              rlPosYofMeIn0 = xydistance;
              break;
            case 2: // right
              rlPosXofMeIn0 = -xydistance;
              rlPosYofMeIn0 = -xydistance;
              break;
            case 3: // back
              rlPosXofMeIn0 = -2*xydistance;
              rlPosYofMeIn0 = 0;
              break;
            default:
              DEBUG_PRINT("myId is not in the range of 0-3\n");
              desireY = rlVarForCtrl[0][STATE_rlX];
              desireX = rlVarForCtrl[0][STATE_rlY];
              moveWithLeaderAsOrigin(desireX, desireY);
              continue;
          }
          // yaw compensation
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