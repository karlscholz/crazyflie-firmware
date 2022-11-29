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
#include "estimator_kalman.h"
#include "cf_math.h"
#include "lpsTwrTag.h"
#include "swarmTwrTag.h"

// the 64 bit uwb base adress the receiver id is added to
#define BASIC_ADDR 0xbccf851300000000
// Id of this dorne, based on the 40 bit radio adress, but only the last hex character(the last 4 bits) are used.
static uint8_t myId;
// The 64 bit adress of this uwb node, BASIC_ADDR + myId
static locoAddress_t myAddr;

// keeps track of the state the module is in and restricts the possible states of txPacket.payload[0]: POLL, ANSWER, FINAL & REPORT
typedef enum {
    transmitter,
    receiver
} ComMode;
static ComMode comMode;

// swarm state containing my attitude and velocitydata as well as the one of every other uwb node, get's retrieved by relative_localization() in relative_localization.c
typedef struct {
  uint16_t distance[NUM_UWB];
  float velX[NUM_UWB];
  float velY[NUM_UWB];
  float gyroZ[NUM_UWB];
  float height[NUM_UWB];
  bool update[NUM_UWB];
  bool fly;
  float myVelX;
  float myVelY;
  float myGyroZ;
  float myHeight;
} swarmInfo_t;
static swarmInfo_t state;

// 64/40 bit timestamps for the different phases of the two way ranging protocol
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;
static packet_t txPacket;
// true if in report case distanceCompute != 0 && median_data[receiverId].pointer == 3
static bool rangingOk;

// Id of the other dorne, based on its 40 bit radio adress, but only the last hex character(the last 4 bits) are used.
static uint8_t receiverId;
// is set to true if a change of the comMode is allowed
static bool comModeTurnCheck;
// keeps track of the time so a check if any uwb nodes becomes transmitter within 20ms is possible
static uint32_t comModeTurnCheckTick = 0;

//array with size NUM_UWB, each element is a struct containing the last 3 distance measurements with the corresponding uwb node (and a pointer to the next element to be overwritten)
typedef struct {
  uint16_t distances[3];
  uint8_t pointer;
} median_data_t;
static median_data_t median_data[NUM_UWB];

/* Updates the velocity and attidude data of the swarmTwrTagReportPayload_t
 * structs report and report2 by retrieving data from the flight EKF.
 *
 * Parameters:
 * - float* velX, float* velY, float* gyroZ, float* height
 * Returns:
 * - Nothing
 *                                                                            */
static void getSwarmInputsFromKalman(float* velX, float* velY, float* gyroZ, float* height){
  // empty R 3x3 matrix for retrieving the "The quad's attitude as a rotation matrix" 
  float R[3][3], gyroZtemp;
  point_t point;
  // retrieve "The quad's attitude as a rotation matrix"
  estimatorKalmanGetEstimatedRot((float*)R);
  // retrieve "The quad's velocity in the world frame"
  estimatorKalmanGetEstimatedVel(&point);
  float PX = point.x;
  float PY = point.y;
  float PZ = point.z;
  // get the height of the quad from the z component of the global position data
  estimatorKalmanGetEstimatedPos(&point);
  *height = point.z;
  // get the yaw rate
  estimatorKalmanGetGyroZ(&gyroZtemp);
  *gyroZ = gyroZtemp;
  // calculate the velocity in the body frame based on the quad's rotation matrix and the velocity in the world frame
  *velX = R[0][0] * PX + R[0][1] * PY + R[0][2] * PZ;
  *velY = R[1][0] * PX + R[1][1] * PY + R[1][2] * PZ;
}

/* Simply gives you the middle value of the three values in a 3 element array
 *
 * Parameters:
 * - uint16_t* data --> vmedian_data[receiverId].distances
 * Returns:
 * - uint16_t middle
 *                                                                            */
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

/* Transmitter mode: sends a POLL or FINAL packet to the receiver or changes
 * mode to receiver.
 * Receiver mode: sends an ANSWER packet to the transmitter
 *
 * Parameters:
 * - dwDevice_t *dev --> of the other uwb node
 * Returns:
 * - Nothing
 *                                                                            */
static void txcallback(dwDevice_t *dev) {
  // timestamp of departure
  dwTime_t departure;
  // get the timestamp of departure from the dw1000 as 40 bits in departure.raw
  dwGetTransmitTimestamp(dev, &departure);
  // add half of the antena delay to the 64 bit full timestamp interpretation of departure union
  departure.full += (LOCODECK_ANTENNA_DELAY / 2);

  if (comMode == transmitter) {
    switch (txPacket.payload[0]) {
      case LPS_TWR_POLL:
        // if mode is TX and case is POLL, set polltx to the timestamp departure
        poll_tx = departure;
        break;
      case LPS_TWR_FINAL:
        // if mode is TX and case is FINAL, set finaltx to the timestamp departure
        final_tx = departure;
        break;
      case LPS_TWR_REPORT + 1:
        if (NUM_UWB > 2) { // Token-ring communication
          if ((receiverId == 0) || (receiverId == myId + 1)) {
            // if case is REPORT+1 (0x04+1=0x05) and (rxID is 0=Leader or rxID is myID+1=NextInLine), change comMode to receiver
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
        // if mode is RX and case is ANSWER, set answertx to the timestamp departure
        answer_tx = departure;
        break;
      case LPS_TWR_REPORT:
        break;
    }
  }
}

/* Transmitter mode: sends a POLL or FINAL packet to the receiver or changes
 * mode to receiver.
 * Receiver mode: sends an ANSWER packet to the transmitter
 *
 * Parameters:
 * - dwDevice_t *dev --> of the other uwb node
 * Returns:
 * - Nothing
 *                                                                            */
static void rxcallback(dwDevice_t *dev) {
  // timestamp of arrival, initiated with the 64 bits zeroed
  dwTime_t arival = { .full=0 };
  // calculate the length of the data in the rxPacket 
  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0)
    return;

  packet_t rxPacket;
  // zero the new packet_t rxPacket
  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  // read the data from the dw1000 in rxPacket
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);
  // if the packet is from myself, exit out, if I'm in transmitter mode, change to receiver mode and go on
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

  // swap receiver and sender for the new packet I'll send based on the just arrived packet
  txPacket.destAddress = rxPacket.sourceAddress;
  txPacket.sourceAddress = rxPacket.destAddress;

  if (comMode == transmitter) {
    switch (rxPacket.payload[LPS_TWR_TYPE]) {
      case LPS_TWR_ANSWER: {
        // if mode is TX and case is ANSWER, set the mode to FINAL, forward sequence information 
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_FINAL;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        // read out the timestamp of arrival to the 40 bits arival.raw and compensate for the antenna delay in the 64 bit full interpretation
        dwGetReceiveTimestamp(dev, &arival);
        // remove half of the antenna delay from the 64 bit full timestamp interpretation of arival union
        arival.full -= (LOCODECK_ANTENNA_DELAY / 2);
        // set the answer_rx to the true timestamp of arrival
        answer_rx = arival;
        // with answer_rx timestamp calculated, send the FINAL packet out
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_REPORT: {
        // if mode is TX and case is REPORT, calculate everything and send the REPORT packet out
        swarmTwrTagReportPayload_t *report = (swarmTwrTagReportPayload_t *)(rxPacket.payload+2);
        // create aux variables for the calculations
        double tround1, treply1, treply2, tround2, tprop_ctn, tprop;
        // copy all 5 uint8_t variables of the report->XYZ array into XYZ dwTime_t union 
        memcpy(&poll_rx, &report->pollRx, 5);
        memcpy(&answer_tx, &report->answerTx, 5);
        memcpy(&final_rx, &report->finalRx, 5);
        // calculate the round trip times
        tround1 = answer_rx.low32 - poll_tx.low32;
        treply1 = answer_tx.low32 - poll_rx.low32;
        tround2 = final_rx.low32 - answer_tx.low32;
        treply2 = final_tx.low32 - answer_rx.low32;
        // calculate the propagation time in timesteps
        tprop_ctn = ((tround1 * tround2) - (treply1 * treply2)) / (tround1 + tround2 + treply1 + treply2);
        // convert the timesteps to seconds
        tprop = tprop_ctn / LOCODECK_TS_FREQ;
        // calculate the distance in millimeters
        uint16_t distanceCompute = (uint16_t)(1000 * (SPEED_OF_LIGHT * tprop + 1));
        if (distanceCompute != 0) {
          uint16_t medianDist = median_filter_3(median_data[receiverId].distances);
          if (ABS(medianDist - distanceCompute) > 500)
            // if the distance is more than 500mm different from the median, don't update the distance, set it to the median distance
            state.distance[receiverId] = medianDist;
          else
            // if the distance is less than 500mm different from the median, update the distance to the new calculated distance
            state.distance[receiverId] = distanceCompute;
          // the median pointer is increased by 1
          median_data[receiverId].pointer++;
          // modulo 3 to keep the pointer in the range 0-2, the position of the newest distance in the array doesn't matter
          if (median_data[receiverId].pointer == 3)
            median_data[receiverId].pointer = 0;
          // the newest, just calculated distance is added to the array
          median_data[receiverId].distances[median_data[receiverId].pointer] = distanceCompute;        
          rangingOk = true;
          // copy the report data of the other drone to the state data, which gets retrieved by getSwarmTwrInfo() from relative_localization.c
          state.velX[receiverId] = report->velX;
          state.velY[receiverId] = report->velY;
          state.gyroZ[receiverId] = report->gyroZ;
          state.height[receiverId]  = report->height;
          if (receiverId == 0)
            // if the packet is from the leader, also update the flyStatus to follow the leader's behavior
            state.fly = report->flyStatus;
          // set the update flag to true, so that the state data gets updated in getSwarmTwrInfo()
          state.update[receiverId] = true;
        }
        // new Payload report2, initialized with ? TODO@KS
        swarmTwrTagReportPayload_t *report2 = (swarmTwrTagReportPayload_t *)(txPacket.payload + 2);
        // change the payload to report +1
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT + 1;
        // forward the sequence
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        // change the distance to the just calculated distance
        report2->distance = distanceCompute;
        // update my own velocity and height in report2 and in my state data
        getSwarmInputsFromKalman(&report2->velX, &report2->velY, &report2->gyroZ, &report2->height);
        state.myVelX = report2->velX;
        state.myVelY = report2->velY;
        state.myGyroZ = report2->gyroZ;
        state.myHeight = report2->height;
        // forward the fly status to the txPacket
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
        // if mode is RX and case is POLL, set the type to answer, forward the sequence
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_ANSWER;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        // read out arrival time step and clean it up
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (LOCODECK_ANTENNA_DELAY / 2);
        poll_rx = arival;
        // poll_rx timestamp calculated, send out ANSWER packet
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2);
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_FINAL: {
        // if mode is RX and case is FINAL, new payload report, initialized with ? TODO@KS
        swarmTwrTagReportPayload_t *report = (swarmTwrTagReportPayload_t *)(txPacket.payload + 2);
        // read out arrival time and clean it up
        dwGetReceiveTimestamp(dev, &arival);
        arival.full -= (LOCODECK_ANTENNA_DELAY / 2);
        final_rx = arival;
        
        txPacket.payload[LPS_TWR_TYPE] = LPS_TWR_REPORT;
        txPacket.payload[LPS_TWR_SEQ] = rxPacket.payload[LPS_TWR_SEQ];
        // copy the three receiverside timestamps in the report
        memcpy(&report->pollRx, &poll_rx, 5);
        memcpy(&report->answerTx, &answer_tx, 5);
        memcpy(&report->finalRx, &final_rx, 5);
        // get the velocity and height from the kalman filter
        getSwarmInputsFromKalman(&report->velX, &report->velY, &report->gyroZ, &report->height);
        state.myVelX = report->velX;
        state.myVelY = report->velY;
        state.myGyroZ = report->gyroZ;
        state.myHeight = report->height;
        report->flyStatus = state.fly;
        // all receiver side timesteps measured, new velocity and height retrieved, send out REPORT packet
        dwNewTransmit(dev);
        dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH + 2 + sizeof(swarmTwrTagReportPayload_t));
        dwWaitForResponse(dev, true);
        dwStartTransmit(dev);
        break;
      }
      case LPS_TWR_REPORT + 1: {
        // if mode is RX and case is REPORT +1, new payload report2, initialized with ? TODO@KS
        swarmTwrTagReportPayload_t *report2 = (swarmTwrTagReportPayload_t *)(rxPacket.payload + 2);
        // create transmitterid from the sender address 
        uint8_t transmitterId = (uint8_t)(rxPacket.sourceAddress & 0xFF);
        if (report2->distance != 0) {
          // the same behaviour as the transmitter drone just did in the REPORT case
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
          // in addition to the REPORT case, rangingOK is also set to true here
          rangingOk = true;
        }
        if ( (NUM_UWB > 2) && ( myId == transmitterId + 1 || myId == 0)) {
            // (see opposite behaviour in txcallback case REPORT+1)
            // if case is REPORT+1 (0x04+1=0x05) and (myID is 0=Leader or myID is txID+1=IAmNextInLine), change comMode to transmitter 
            comMode = transmitter;
            dwIdle(dev);
            dwSetReceiveWaitTimeout(dev, 1000);
            // after changing role to transmitter, choose the new drone to contact
            if (myId == NUM_UWB - 1)
              // if myID is the last drone, contact the first drone
              receiverId = 0;
            else
              // if myID is not the last drone, contact the last drone
              receiverId = NUM_UWB - 1;
            if (myId == 0)
              // if myID is the leader, contact the the second to the last drone
              receiverId = NUM_UWB - 2;
            // send out the first POLL packet
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

bool getSwarmTwrInfo(int agentId, uint16_t* distance, float* velX, float* velY, float* gyroZ, float* height, float* myVelX, float* myVelY, float* myGyroZ, float* myHeight) {
  if (state.update[agentId] == true) {
    state.update[agentId] = false;
    *distance = state.distance[agentId];
    *velX = state.velX[agentId];
    *velY = state.velY[agentId];
    *gyroZ = state.gyroZ[agentId];
    *height = state.height[agentId];
    *myVelX = state.myVelX;
    *myVelY = state.myVelY;
    *myGyroZ = state.myGyroZ;
    *myHeight = state.myHeight;
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
LOG_GROUP_STOP(ranging)