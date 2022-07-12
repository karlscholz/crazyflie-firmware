/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * LPS node firmware.
 *
 * Copyright 2016, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Foobar is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
 */
/* swarmTwrTag.c: Uwb two way ranging inter swarms of Crazyflies, Shushuai Li */


#include <string.h>
#include "lpsTwrTag.h"
#include "log.h"
#include "physicalConstants.h"
#include "task.h"
#include "configblock.h"
// #include "estimator_kalman.h"
#include "lpsTwrTag.h"
#include "swarmTwrTag.h"

#define BASIC_ADDR 0xbccf851300000000
static uint8_t myId;
static locoAddress_t myAddr;

typedef enum {
    transmitter,
    receiver
} ComMode;
static ComMode comMode;

typedef struct {
  uint16_t distance[NUM_UWB];
  float velX[NUM_UWB];
  float velY[NUM_UWB];
  float gyroZ[NUM_UWB];
  float height[NUM_UWB];
  bool update[NUM_UWB];
  bool fly;
} swarmInfo_t;
static swarmInfo_t state;

static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;
static packet_t txPacket;
static bool rangingOk;

static uint8_t receiverId;
static bool comModeTurnCheck;
static uint32_t comModeTurnCheckTick = 0;

typedef struct {
  uint16_t distances[3];
  uint8_t pointer;
} median_data_t;
static median_data_t median_data[NUM_UWB];

static uint16_t median_filter_3(uint16_t* data) {
  uint16_t middle;
  if ((data[0] <= data[1]) && (data[0] <= data[2])) {
    middle = (data[1] <= data[2]) ? data[1] : data[2];
  }
  else if ((data[1] <= data[0]) && (data[1] <= data[2])) {
    middle = (data[0] <= data[2]) ? data[0] : data[2];
  }
  else {
    middle = (data[0] <= data[1]) ? data[0] : data[1];
  }
  return middle;
}
#define ABS(a) ((a) > 0 ? (a) : -(a))

static void txcallback(dwDevice_t *dev) {
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (LOCODECK_ANTENNA_DELAY / 2);

  if (comMode == transmitter) {
    switch (txPacket.payload[0]) {
      case LPS_TWR_POLL:
        poll_tx = departure;
        break;
      case LPS_TWR_FINAL:
        final_tx = departure;
        break;
      case LPS_TWR_REPORT + 1:
        if (NUM_UWB > 2) { // Token-ring communication
          if ((receiverId == 0) || (receiverId == myId + 1)) {
            comMode = receiver;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 10000);
            dwNewReceive(dev);
            dwSetDefaults(dev);
            dwStartReceive(dev);
            comModeTurnCheck = true;
            comModeTurnCheckTick = xTaskGetTickCount();
          } else {
            receiverId = receiverId - 1;
          }
        }
        break;
    }
  } else {
    switch (txPacket.payload[0]) {
      case LPS_TWR_ANSWER:
        answer_tx = departure;
        break;
      case LPS_TWR_REPORT:
        break;
    }
  }
}

static void rxcallback(dwDevice_t *dev) {
  dwTime_t arival = { .full=0 };
  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0)
    return;

  packet_t rxPacket;
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  if (rxPacket.destAddress != myAddr) {
    if (comMode == transmitter) {
      // mode change is not successful
      comMode = receiver;
      dwIdle(dev);
      dwSetReceiveWaitTimeout(dev, 10000);
    }
    dwNewReceive(dev);
    dwSetDefaults(dev);
    dwStartReceive(dev);
    return;
  }

  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  if (comMode == transmitter) {
    switch (rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_ANSWER: {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (LOCODECK_ANTENNA_DELAY / 2);
        answer_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_REPORT: {
        swarmTwrTagReportPayload_t *report = (swarmTwrTagReportPayload_t *)(rxPacket.payload+2);
        double tround1, treply1, treply2, tround2, tprop_ctn, tprop;
        memcpy(&poll_rx, &report->pollRx, 5);
        memcpy(&answer_tx, &report->answerTx, 5);
        memcpy(&final_rx, &report->finalRx, 5);
        tround1 = answer_rx.low32 - poll_tx.low32;
        treply1 = answer_tx.low32 - poll_rx.low32;
        tround2 = final_rx.low32 - answer_tx.low32;
        treply2 = final_tx.low32 - answer_rx.low32;
        tprop_ctn = ((tround1 * tround2) - (treply1 * treply2)) / (tround1 + tround2 + treply1 + treply2);
        tprop = tprop_ctn / LOCODECK_TS_FREQ;
        uint16_t distanceCompute = (uint16_t)(1000 * (SPEED_OF_LIGHT * tprop + 1));
        if (distanceCompute != 0) {
          uint16_t medianDist = median_filter_3(median_data[receiverId].distances);
          if (ABS(medianDist - distanceCompute) > 500)
            state.distance[receiverId] = medianDist;
          else
            state.distance[receiverId] = distanceCompute;
          median_data[receiverId].pointer++;
          if (median_data[receiverId].pointer == 3)
            median_data[receiverId].pointer = 0;
          median_data[receiverId].distances[median_data[receiverId].pointer] = distanceCompute;        
          rangingOk = true;
          state.velX[receiverId] = report->velX;
          state.velY[receiverId] = report->velY;
          state.gyroZ[receiverId] = report->gyroZ;
          state.height[receiverId]  = report->height;
          if (receiverId == 0)
            state.fly = report->flyStatus;
          state.update[receiverId] = true;
        }
        swarmTwrTagReportPayload_t *report2 = (swarmTwrTagReportPayload_t *)(txPacket.payload + 2);
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT + 1;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        report2->distance = distanceCompute;
        // estimatorKalmanGetSwarmInfo(&report2->velX, &report2->velY, &report2->gyroZ, &report2->height);
        report2->flyStatus = state.fly;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(swarmTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
    }
  } else {
    switch (rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_POLL: {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (LOCODECK_ANTENNA_DELAY / 2);
        poll_rx = arival;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_FINAL: {
        swarmTwrTagReportPayload_t *report = (swarmTwrTagReportPayload_t *)(txPacket.payload + 2);
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (LOCODECK_ANTENNA_DELAY / 2);
        final_rx = arival;
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        memcpy(&report->pollRx, &poll_rx, 5);
        memcpy(&report->answerTx, &answer_tx, 5);
        memcpy(&report->finalRx, &final_rx, 5);
        // estimatorKalmanGetSwarmInfo(&report->velX, &report->velY, &report->gyroZ, &report->height);
        report->flyStatus = state.fly;
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(swarmTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_REPORT + 1: {
        swarmTwrTagReportPayload_t *report2 = (swarmTwrTagReportPayload_t *)(rxPacket.payload + 2);
        uint8_t transmitterId = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        if (report2->distance != 0) {
          uint16_t distanceCompute = report2->distance;
          uint16_t medianDist = median_filter_3(median_data[transmitterId].distances);
          if (ABS(medianDist-distanceCompute) > 500)
            state.distance[transmitterId] = medianDist;
          else
            state.distance[transmitterId] = distanceCompute;
          median_data[transmitterId].pointer++;
          if (median_data[transmitterId].pointer == 3)
            median_data[transmitterId].pointer = 0;
          median_data[transmitterId].distances[median_data[transmitterId].pointer] = distanceCompute; 
          state.velX[transmitterId] = report2->velX;
          state.velY[transmitterId] = report2->velY;
          state.gyroZ[transmitterId] = report2->gyroZ;
          state.height[transmitterId]  = report2->height;
          if (transmitterId == 0)
            state.fly = report2->flyStatus;
          state.update[transmitterId] = true;
          rangingOk = true;
        }
        if ( (NUM_UWB > 2) && ( myId == transmitterId + 1 || myId == 0)) {
            comMode = transmitter;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            if (myId == NUM_UWB - 1)
              receiverId = 0;
            else
              receiverId = NUM_UWB - 1;
            if (myId == 0)
              receiverId = NUM_UWB - 2;
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = myAddr;
            txPacket.destAddress = BASIC_ADDR + receiverId;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
        } else {
          dwNewReceive(dev);
          dwSetDefaults(dev);
          dwStartReceive(dev);
        }
        break;
      }
    }
  }
}

static uint32_t twrTagOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
  switch (event) {
    case eventPacketReceived:
      rxcallback(dev);
      if (NUM_UWB > 2)
        comModeTurnCheck = false;
      break;
    case eventPacketSent:
      txcallback(dev);
      break;
    case eventTimeout:
    case eventReceiveTimeout:
    case eventReceiveFailed:
      if (comMode == transmitter) {
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
        txPacket.payload[LPS_TWR_SEQ] = 0;
        txPacket.sourceAddress = myAddr;
        txPacket.destAddress = BASIC_ADDR + receiverId;
        dwNewTransmit(dev);
        dwSetDefaults(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
      } else {
        if (xTaskGetTickCount() > comModeTurnCheckTick + 20 && NUM_UWB > 2) {
          // check if any uwb becomes transmitter within 20ms
          if (comModeTurnCheck == true) {
            comMode = transmitter;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_POLL;
            txPacket.payload[LPS_TWR_SEQ] = 0;
            txPacket.sourceAddress = myAddr;
            txPacket.destAddress = BASIC_ADDR + receiverId;
            dwNewTransmit(dev);
            dwSetDefaults(dev);
            dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);
            dwWaitForResponse(dev, true);
            dwStartTransmit(dev);
            comModeTurnCheck = false;
            break;
          }
        }
        dwNewReceive(dev);
	      dwSetDefaults(dev);
        dwStartReceive(dev);
      }     
      break;
    default:
      configASSERT(false);
  }
  return MAX_TIMEOUT;
}

static void twrTagInit(dwDevice_t *dev)
{
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));

  myId = (uint8_t)((configblockGetRadioAddress()) & 0x000000000f);
  myAddr = BASIC_ADDR + myId;

  if (myId == 0) {
    receiverId = NUM_UWB - 1;
    comMode = transmitter;
    dwSetReceiveWaitTimeout(dev, 1000);
  } else {
    comMode = receiver;
    dwSetReceiveWaitTimeout(dev, 10000);
  }

  for (int i = 0; i < NUM_UWB; i++) {
    median_data[i].pointer = 0;
    state.update[i] = false;
  }

  state.fly = false;
  if (NUM_UWB > 2)
    comModeTurnCheck = false;
  rangingOk = false;
}

static bool isRangingOk() {
  return rangingOk;
}

static bool getAnchorPosition(const uint8_t anchorId, point_t* position) {
  if (anchorId < NUM_UWB-1)
    return true;
  else
    return false;
}

static uint8_t getAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  for (int i = 0; i < NUM_UWB-1; i++) {
    unorderedAnchorList[i] = i;
  }
  return NUM_UWB-1;
}

static uint8_t getActiveAnchorIdList(uint8_t unorderedAnchorList[], const int maxListSize) {
  uint8_t count = 0;
  for (int i = 0; i < NUM_UWB-1; i++) {
      unorderedAnchorList[count] = i;
      count++;
  }
  return count;
}

bool getSwarmTwrInfo(int agentId, uint16_t* distance, float* velX, float* velY, float* gyroZ, float* height) {
  if (state.update[agentId] == true) {
    state.update[agentId] = false;
    *distance = state.distance[agentId];
    *velX = state.velX[agentId];
    *velY = state.velY[agentId];
    *gyroZ = state.gyroZ[agentId];
    *height = state.height[agentId];
    return true;
  } else
    return false;
}

bool updateFlyStatus(int agentId, bool flyStatus) {
  if( agentId == 0) {
    state.fly = flyStatus;
    return flyStatus;
  } else
    return state.fly;
}

uwbAlgorithm_t uwbTwrTagAlgorithm = {
  .init = twrTagInit,
  .onEvent = twrTagOnEvent,
  .isRangingOk = isRangingOk,
  .getAnchorPosition = getAnchorPosition,
  .getAnchorIdList = getAnchorIdList,
  .getActiveAnchorIdList = getActiveAnchorIdList,
};

LOG_GROUP_START(ranging)
LOG_ADD(LOG_UINT16, distance0, &state.distance[0])
LOG_ADD(LOG_UINT16, distance1, &state.distance[1])
LOG_ADD(LOG_UINT16, distance2, &state.distance[2])
LOG_ADD(LOG_UINT16, distance3, &state.distance[3])
LOG_ADD(LOG_UINT16, distance4, &state.distance[4])
LOG_GROUP_STOP(ranging)