#ifndef __SWARM_TWR_TAG_H__
#define __SWARM_TWR_TAG_H__

#include "locodeck.h"

#define NUM_UWB 4
#define MANUAL_CONTROL_LEADER

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];
  uint16_t distance;
  float velX;
  float velY;
  float gyroZ;
  float height;
  bool flyStatus;
} swarmTwrTagReportPayload_t;

bool getSwarmTwrInfo(int agentId, uint16_t* distance, float* velX, float* velY, float* gyroZ, float* height, float* myVelX, float* myVelY, float* myGyroZ, float* myHeight);
bool updateFlyStatus(int agentId, bool flyStatus);

#endif // __SWARM_TWR_TAG_H__
