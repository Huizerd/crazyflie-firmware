#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "uart2.h"
#include "log.h"
#include "debug.h"
#include <math.h>

#define DEBUG_MODULE "MAVIC"

#define SERIAL_HEADER 0x00

static bool isInit;

// Flight plan: square in cyberzoo
/**
 * TODO: for crazyflie, we need landing point, for mavic, we dont!
 */
#define FPLENGTH 7
static float flightPlan[FPLENGTH][3] = {{0.0f, 0.0f, 1.0f},
                               {-1.0f, -1.0f, 1.0f},
                               {-1.0f, 1.0f, 1.0f},
                               {1.0f, 1.0f, 1.0f},
                               {1.0f, -1.0f, 1.0f},
                               {-1.0f, -1.0f, 1.0f},
                               {0.0f, 0.0f, 1.0f}};

typedef struct __attribute__((packed)) mavicPacket_s {
  uint8_t header;
  float x;
  float y;
  float z;
} mavicPacket_t;

typedef struct mavicData_s {
  float x;
  float y;
  float z;
} mavicData_t;



static mavicData_t currentPos;
static mavicData_t desiredPos;
static mavicData_t command;

void mavicTask(void *param)
{
  int velXid;
  int velYid;
  int velZid;
  int posXid;
  int posYid;
  int posZid;

  mavicPacket_t packet;

  systemWaitStart();

  // Setup data to transfer
  velXid = logGetVarId("posCtl", "targetVX");
  velYid = logGetVarId("posCtl", "targetVY");
  velZid = logGetVarId("posCtl", "targetVZ");
  posXid = logGetVarId("UWB2POS", "estX");
  posYid = logGetVarId("UWB2POS", "estY");
  posZid = logGetVarId("UWB2POS", "estZ");

  TickType_t lastWakeTime = xTaskGetTickCount();

  int idx = 0;

  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(10));

    // Assemble the data
    packet.header = SERIAL_HEADER;
    packet.x = logGetFloat(velXid);
    packet.y = logGetFloat(velYid);
    packet.z = logGetFloat(velZid);

    // Get current multilaterated position
    currentPos.x = logGetFloat(posXid);
    currentPos.y = logGetFloat(posYid);
    currentPos.z = logGetFloat(posZid);

    // Get current desired position
    desiredPos.x = flightPlan[idx][0];
    desiredPos.y = flightPlan[idx][1];
    desiredPos.z = flightPlan[idx][2];

    // Get velocity command based on proportional control with gain of 2 clamped to (-1, 1)
    command.x = fmaxf(-1.0f, fminf(1.0f, 2.0f * (desiredPos.x - currentPos.x)));
    command.y = fmaxf(-1.0f, fminf(1.0f, 2.0f * (desiredPos.y - currentPos.y)));
    command.z = 0.0f;

    // Next checkpoint
    if (fabsf(desiredPos.x - currentPos.x) < 0.1f && fabsf(desiredPos.y - currentPos.y) < 0.1f && idx < (FPLENGTH - 1))
      idx++;

    // Send the three floats, byte by byte, to UART2
    uart2SendDataDmaBlocking(sizeof(mavicData_t), (uint8_t *)(&packet));
  }
}


static void mavicInit()
{
  DEBUG_PRINT("Starting Mavic task: writing velocity commands to UART2\n");

  xTaskCreate(mavicTask, "MAVIC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // Configure uart and set the baud rate
  /**
   * TODO: here or in cf-board.mk as flag?
   */
  uart2Init(115200);

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


LOG_GROUP_START(mavic)
  LOG_ADD(LOG_FLOAT, currentPosX, &currentPos.x)
  LOG_ADD(LOG_FLOAT, currentPosY, &currentPos.y)
  LOG_ADD(LOG_FLOAT, currentPosZ, &currentPos.z)
  LOG_ADD(LOG_FLOAT, desiredPosX, &desiredPos.x)
  LOG_ADD(LOG_FLOAT, desiredPosY, &desiredPos.y)
  LOG_ADD(LOG_FLOAT, desiredPosZ, &desiredPos.z)
  LOG_ADD(LOG_FLOAT, commandX, &command.x)
  LOG_ADD(LOG_FLOAT, commandY, &command.y)
  LOG_ADD(LOG_FLOAT, commandZ, &command.z)
LOG_GROUP_STOP(mavic)
