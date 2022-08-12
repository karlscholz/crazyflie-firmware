/**
 * relative_localization.c - UWB based inter-drone relative position estimation.
 * The crazyflie has to have the locodeck and the flowdeck version 2.
 * 
 * Project page: https://shushuai3.github.io/swarm.html
 * Paper: https://arxiv.org/abs/2003.05853
 */

#include "arm_math.h"
#include "FreeRTOS.h"
#include "task.h"
#include "param.h"
#include "log.h"
#include "swarmTwrTag.h"
#include "relative_localization.h"
#include "cf_math.h"
#include "debug.h"

static bool is_init = false;

// deviation of velocity, yaw rate, distance; and covariance
static float Qv = 1.0f;
static float Qr = 0.7f;
static float Ruwb = 2.0f;
static float InitCovPos = 1000.0f;
static float InitCovYaw = 1.0f;

static rlState_t rlState[NUM_UWB];

static float A[STATE_DIM_rl][STATE_DIM_rl];
static float h[STATE_DIM_rl] = {0.0f};
static arm_matrix_instance_f32 H = {1, STATE_DIM_rl, h};
static arm_matrix_instance_f32 Am = { STATE_DIM_rl, STATE_DIM_rl, (float *)A};

// check if communication is lost
static bool comConnection = false;
static int8_t comCount = 0;

// temporary matrices for the covariance updates
static float tmpNN1d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN1m = { STATE_DIM_rl, STATE_DIM_rl, tmpNN1d};
static float tmpNN2d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN2m = { STATE_DIM_rl, STATE_DIM_rl, tmpNN2d};
static float K[STATE_DIM_rl];
static arm_matrix_instance_f32 Km = {STATE_DIM_rl, 1, (float *)K};
static float tmpNN3d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN3d};
static float HTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 HTm = {STATE_DIM_rl, 1, HTd};
static float PHTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 PHTm = {STATE_DIM_rl, 1, PHTd};

void relativeEKF(int n, float vxi, float vyi, float ri, float hi, float vxj, float vyj, float rj, float hj, uint16_t dij, float dt) {
  arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *)rlState[n].P};
  float cyaw = arm_cos_f32(rlState[n].S[STATE_rlYaw]);
  float syaw = arm_sin_f32(rlState[n].S[STATE_rlYaw]);
  float xij = rlState[n].S[STATE_rlX];
  float yij = rlState[n].S[STATE_rlY];

  // prediction
  rlState[n].S[STATE_rlX] = xij + (cyaw * vxj - syaw * vyj - vxi + ri * yij) * dt;
  rlState[n].S[STATE_rlY] = yij + (syaw * vxj + cyaw * vyj - vyi - ri * xij) * dt;
  rlState[n].S[STATE_rlYaw] = rlState[n].S[STATE_rlYaw] + (rj - ri) * dt;

  A[0][0] = 1;
  A[0][1] = ri * dt;
  A[0][2] = (-syaw * vxj - cyaw * vyj) * dt;
  A[1][0] = -ri * dt;
  A[1][1] = 1;
  A[1][2] = (cyaw * vxj - syaw * vyj) * dt;
  A[2][0] = 0;
  A[2][1] = 0;
  A[2][2] = 1;

  mat_mult(&Am, &Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'

  // BQB' = [ Qv*c^2 + Qv*s^2 + Qr*y^2 + Qv,                       -Qr*x*y, -Qr*y]
  //        [                       -Qr*x*y, Qv*c^2 + Qv*s^2 + Qr*x^2 + Qv,  Qr*x]
  //        [                         -Qr*y,                          Qr*x,  2*Qr]*dt^2
  float dt2 = dt * dt;
  rlState[n].P[0][0] += dt2 * (Qv + Qv + Qr * yij * yij);
  rlState[n].P[0][1] += dt2 * (-Qr * xij * yij);
  rlState[n].P[0][2] += dt2 * (-Qr * yij);
  rlState[n].P[1][0] += dt2 * (-Qr * xij * yij);
  rlState[n].P[1][1] += dt2 * (Qv + Qv + Qr * xij * xij);
  rlState[n].P[1][2] += dt2 * (Qr * xij);
  rlState[n].P[2][0] += dt2 * (-Qr * yij);
  rlState[n].P[2][1] += dt2 * (Qr * xij);
  rlState[n].P[2][2] += dt2 * (2 * Qr);

  xij = rlState[n].S[STATE_rlX];
  yij = rlState[n].S[STATE_rlY];
  float distPred = sqrtf(xij * xij + yij * yij + (hi - hj) * (hi - hj)) + 0.0001f;
  float distMeas = (float)(dij / 1000.0f);
  // UWB bias model (fitting with optiTrack measurement)
  distMeas = distMeas - (0.048f * distMeas + 0.65f);
  h[0] = xij / distPred;
  h[1] = yij / distPred;
  h[2] = 0;

  mat_trans(&H, &HTm); // H'
  mat_mult(&Pm, &HTm, &PHTm); // PH'
  float HPHR = powf(Ruwb, 2); // HPH' + R
  for (int i = 0; i < STATE_DIM_rl; i++) {
    HPHR += H.pData[i]*PHTd[i];
  }
  for (int i = 0; i < STATE_DIM_rl; i++) {
    K[i] = PHTd[i] / HPHR; // kalman gain = (PH'(HPH' + R )^-1)
    rlState[n].S[i] = rlState[n].S[i] + K[i] * (distMeas - distPred); // state update
  }
  mat_mult(&Km, &H, &tmpNN1m); // KH
  for (int i = 0; i < STATE_DIM_rl; i++) {
    tmpNN1d[STATE_DIM_rl*i+i] -= 1;
  } // KH - I
  mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I) * P
  mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I) * P * (KH - I)'
}

bool relative_localization(float* rlStateForControl) {
  if (!is_init) {
    for (int n = 0; n < NUM_UWB; n++) {
      for (int i = 0; i < STATE_DIM_rl; i++) {
        for (int j = 0; j < STATE_DIM_rl; j++) {
          rlState[n].P[i][j] = 0;
        }
      }
      rlState[n].P[STATE_rlX][STATE_rlX] = InitCovPos;
      rlState[n].P[STATE_rlY][STATE_rlY] = InitCovPos;
      rlState[n].P[STATE_rlYaw][STATE_rlYaw] = InitCovYaw;  
      rlState[n].S[STATE_rlX] = 0;
      rlState[n].S[STATE_rlY] = 0;
      rlState[n].S[STATE_rlYaw] = 0;
      rlState[n].firstTime = true;
      is_init = true;
    }
  }

  // vel and yaw rate of j-th neighbor; i mean my; h is height
  float vxj, vyj, rj, hi, vxi, vyi, ri, hj;
  uint16_t dij;

  for (int n = 0; n < NUM_UWB; n++) {
    if (getSwarmTwrInfo(n, &dij, &vxj, &vyj, &rj, &hj, &vxi, &vyi, &ri, &hi)) {
      comCount = 0;
      if(!rlState[n].firstTime) {
        uint32_t osTick = xTaskGetTickCount();
        float dtEKF = (float)(osTick - rlState[n].lastTimetick) / configTICK_RATE_HZ;
        rlState[n].lastTimetick = osTick;
        // DEBUG_PRINT("vxi: %f, vyi: %f, ri: %f, hi: %f\n", (double)vxi, (double)vyi, (double)ri, (double)hi);
        // DEBUG_PRINT("vxj: %f, vyj: %f, rj: %f, hj: %f\n", (double)vxj, (double)vyj, (double)rj, (double)hj);
        relativeEKF(n, vxi, vyi, ri, hi, vxj, vyj, rj, hj, dij, dtEKF);
      } else {
        rlState[n].lastTimetick = xTaskGetTickCount();
        rlState[n].firstTime = false; // to skip the first infinite update time
        comConnection = true;
      }
    }
  }
  comCount++;
  // disable flight if com is lost for 1 second
  if (comCount > 100) {
    comConnection = false;
  }

  if(comConnection) {
    for (int i = 0; i < NUM_UWB; i++) {
      *(rlStateForControl + i * STATE_DIM_rl + 0) = rlState[i].S[STATE_rlX];
      *(rlStateForControl + i * STATE_DIM_rl + 1) = rlState[i].S[STATE_rlY];
      *(rlStateForControl + i * STATE_DIM_rl + 2) = rlState[i].S[STATE_rlYaw];
    }   
    return true;
  } else {
    return false;
  }
}

LOG_GROUP_START(rl_state)
LOG_ADD(LOG_FLOAT, rlX0, &rlState[0].S[STATE_rlX])
LOG_ADD(LOG_FLOAT, rlY0, &rlState[0].S[STATE_rlY])
LOG_ADD(LOG_FLOAT, rlYaw0, &rlState[0].S[STATE_rlYaw])
LOG_ADD(LOG_FLOAT, rlX1, &rlState[1].S[STATE_rlX])
LOG_ADD(LOG_FLOAT, rlY1, &rlState[1].S[STATE_rlY])
LOG_ADD(LOG_FLOAT, rlYaw1, &rlState[1].S[STATE_rlYaw])
LOG_GROUP_STOP(rl_state)

PARAM_GROUP_START(rl_param)
PARAM_ADD(PARAM_FLOAT, noiFlow, &Qv) // make sure the name is not too long
PARAM_ADD(PARAM_FLOAT, noiGyroZ, &Qr)
PARAM_ADD(PARAM_FLOAT, noiUWB, &Ruwb)
PARAM_ADD(PARAM_FLOAT, Ppos, &InitCovPos)
PARAM_ADD(PARAM_FLOAT, Pyaw, &InitCovYaw)
PARAM_GROUP_STOP(rl_param)