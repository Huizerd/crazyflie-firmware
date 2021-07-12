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

typedef struct __attribute__((packed)) mavicPacket_s {
  uint8_t header;
  float disp;
} mavicPacket_t;

typedef struct mavicData_s {
  float disp;
} mavicData_t;

static mavicData_t currentDisp;


void mavicTask(void *param)
{
  mavicPacket_t packet;

  systemWaitStart();

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(10));

    // Get float data byte by byte from UART2
    uart2GetDataWithDefaultTimeout((uint8_t *)(&packet));

    // Write data to log variable
    currentDisp.disp = packet.disp;
  }
}


static void mavicInit()
{
  DEBUG_PRINT("Starting Mavic task: getting disparity values from UART2\n");

  xTaskCreate(mavicTask, "MAVIC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

  // Configure uart and set the baud rate
  /**
   * TODO: here or in cf2.mk as flag?
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
  LOG_ADD(LOG_FLOAT, currentDisp, &currentDisp.disp)
LOG_GROUP_STOP(mavic)
