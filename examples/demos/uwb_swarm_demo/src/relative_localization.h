#ifndef RELATIVELOCALIZATION_H_
#define RELATIVELOCALIZATION_H_
#include "system.h"

typedef enum {
  STATE_rlX, STATE_rlY, STATE_rlYaw, STATE_DIM_rl
} rlKalmanStateIdx;

typedef enum {
  INPUT_vxi, INPUT_vyi, INPUT_ri, INPUT_vxj, INPUT_vyj, INPUT_rj, INPUT_DIM
} rlKalmanInputIdx;

typedef struct {
  float S[STATE_DIM_rl];
  float P[STATE_DIM_rl][STATE_DIM_rl];
  uint32_t lastTimetick;
  bool firstTime;
} rlState_t;

bool relative_localization(float* rlStateForControl);
#endif