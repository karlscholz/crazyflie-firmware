#ifndef RELATIVELOCALIZATION_H_
#define RELATIVELOCALIZATION_H_
#include "system.h"

// enum for swarm.c array rlVarForCtrl and relative_localization.c array rlState
typedef enum {
  STATE_rlX, STATE_rlY, STATE_rlYaw, STATE_DIM_rl
} rlKalmanStateIdx;

// idk, not used in the entire repository
typedef enum {
  INPUT_vxi, INPUT_vyi, INPUT_ri, INPUT_vxj, INPUT_vyj, INPUT_rj, INPUT_DIM
} rlKalmanInputIdx;

// the datatype for array rlState with size NUM_UWB
typedef struct {
  // array for X Y and YAW
  float S[STATE_DIM_rl];
  // error covariance matrix --> 2d array for X Y and YAW
  float P[STATE_DIM_rl][STATE_DIM_rl];
  // timestamp of last EKF loop to calculate time difference 
  uint32_t lastTimetick;
  // to only get a lastTimetick in the first loop without running relativeEKF()
  bool firstTime;
} rlState_t;

/* The relative localization algorithm, prepares and controls the relativeEKF()
 *
 * Parameters:
 * - float* rlStateForControl, 2D-float array rlVarForCtrl of swarm.c with all UWB Nodes in one and X, Y, YAW in the other direction
 * Returns:
 * - bool false if comConnection is lost
 *                                                                            */
bool relative_localization(float* rlStateForControl);
#endif