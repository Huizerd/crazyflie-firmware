#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "uart2.h"
#include "log.h"
#include "debug.h"
#include "commander.h"
#include <math.h>

#define DEBUG_MODULE "MAVIC"

#define POSITION_CONTROL_P_GAIN 2.0f

#define MAVIC_CMD_UNSCALED_MAX 1.0f
#define MAVIC_CMD_SCALED_START 0x00
#define MAVIC_CMD_SCALED_STEPS 127   // Steps in one direction, e.g positive x

#define SERIAL_HEADER 0xFF

#define FPLENGTH 7
static float flightPlan[FPLENGTH][3] = {{0.0f, 0.0f, 1.0f},
                               {-1.0f, -1.0f, 1.0f},
                               {-1.0f, 1.0f, 1.0f},
                               {1.0f, 1.0f, 1.0f},
                               {1.0f, -1.0f, 1.0f},
                               {-1.0f, -1.0f, 1.0f},
                               {0.0f, 0.0f, 1.0f}};

typedef struct ctrlPacket_s {
  uint8_t header;
  uint8_t cmdZ;
  uint8_t cmdY;
  uint8_t cmdX;
  uint8_t cmdYaw;
} ctrlPacket_t;

static bool isInit;

static point_t currentPos;
static point_t delta;
static point_t command;

uint8_t scaleCommand(float unscaled){
  unscaled = fmaxf(-MAVIC_CMD_UNSCALED_MAX, fminf(MAVIC_CMD_UNSCALED_MAX, unscaled));
  return (uint8_t) roundf((unscaled + 1.0f) * MAVIC_CMD_SCALED_STEPS); 
}


void mavicTask(void *param)
{
  int posXid;
  int posYid;
  int posZid;
  
  ctrlPacket_t packet;

  systemWaitStart();

  // Get log ids for current position
  posXid = logGetVarId("UWB2POS", "exX");
  posYid = logGetVarId("UWB2POS", "exY");
  posZid = logGetVarId("UWB2POS", "exZ");

  TickType_t lastWakeTime = xTaskGetTickCount();

  int idx = 0;

  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(100)); 
    
    // Get current multilaterated position
    currentPos.x = logGetFloat(posXid);
    currentPos.y = logGetFloat(posYid);
    currentPos.z = logGetFloat(posZid);

    // Calculate error to current goal
    delta.x = flightPlan[idx][0] - currentPos.x;
    delta.y = flightPlan[idx][1] - currentPos.y;
    delta.z = flightPlan[idx][2] - currentPos.z;
    
    if (fabsf(delta.x)<0.1f && fabsf(delta.y)<0.1f && idx<(FPLENGTH-1)){
      // Update current goal
      idx++;
      delta.x = flightPlan[idx][0] - currentPos.x;
      delta.y = flightPlan[idx][1] - currentPos.y;
      delta.z = flightPlan[idx][2] - currentPos.z;
    }

    // Get velocity command based on proportional control with gain of 2, clamped to (-1,1)
    command.x = POSITION_CONTROL_P_GAIN * delta.x;
    command.y = POSITION_CONTROL_P_GAIN * delta.y;
    command.z = 0;

    // Assemble the data
    packet.header = SERIAL_HEADER;
    packet.cmdZ = scaleCommand(command.z);
    packet.cmdY = scaleCommand(command.y);
    packet.cmdX = scaleCommand(command.x);
    packet.cmdYaw = scaleCommand(0.0);   

    // Send the three floats, byte by byte, to UART2
    uart2SendDataDmaBlocking(sizeof(ctrlPacket_t), (uint8_t *)(&packet));
  }
}


static void mavicInit()
{
  DEBUG_PRINT("Starting Mavic task: writing velocity commands to UART2\n");

  uart2Init(115200);
  xTaskCreate(mavicTask, "MAVIC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  isInit = true;
}


static bool mavicTest()
{
  return isInit;
}


static const DeckDriver mavicDriver = {
  .name = "mavic",
  .init = mavicInit,
  .test = mavicTest,
};


DECK_DRIVER(mavicDriver);