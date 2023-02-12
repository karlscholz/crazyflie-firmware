#ifndef __SWARM_TWR_TAG_H__
#define __SWARM_TWR_TAG_H__

#include "locodeck.h"

// the amount of quadrotors you want to fly with, inclunding the leader
#define NUM_UWB 2
// if MANUAL_CONTROL_LEADER flag is set, and the last hex character is 0, the leader is actively controlled via cfclient software
#define MANUAL_CONTROL_LEADER

// struct containing TWR as well as attitude information
typedef struct {
  // 40 bits for dwTime_t
  uint8_t pollRx[5];
  // 40 bits for dwTime_t
  uint8_t answerTx[5];
  // 40 bits for dwTime_t
  uint8_t finalRx[5];
  // gets calculated int case LPS_TWR_REPORT, variable distanceCompute
  uint16_t distance;
  // gets written by getSwarmInputsFromKalman()
  float velX;
  float velY;
  float gyroZ;
  float height;
  // retrieved from state swarmInfo_t
  bool flyStatus;
} swarmTwrTagReportPayload_t;

/* If state.update[agentId] is true, it changes state.update[agentId] to false
 * and overwrites all parameters passed as pointers with corresponding 
 * variables of swarmInfo_t state.
 * (Called from relative_localization.c.)
 *
 * Parameters: 
 * - int agentId
 * - pointers for uwb Distance and attitude variables
 *   first 4 are for other agent, last 4 are for myself
 * Returns:
 * - bool state.update[agentId]
 *                                                                            */
bool getSwarmTwrInfo(int agentId, uint16_t* distance, float* velX, float* velY, float* gyroZ, float* height, float* myVelX, float* myVelY, float* myGyroZ, float* myHeight);

/* Returns bool fly from swarmInfo_t state.
 * If called by leader, writes state.fly to keepFlying beforehand.
 *
 * Parameters:
 * - int agentId, based on the 40 bit radio adress
 * - bool flyStatus, keepFlying in swarm.c
 * Returns:
 * - bool fly from state swarmInfo_t state
 *                                                                            */
bool updateFlyStatus(int agentId, bool flyStatus);

#endif // __SWARM_TWR_TAG_H__
