#include "deck.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"
#include "uart2.h"
#include "log.h"
#include "debug.h"
#include "commander.h"
#include "estimator_mhe.h"
#include <math.h>

#define DEBUG_MODULE "MAVIC"

#define POSITION_CONTROL_P_GAIN 1.0f

#define MAVIC_CMD_UNSCALED_MAX 1.0f
#define MAVIC_CMD_SCALED_POS (127 + 60)
#define MAVIC_CMD_SCALED_NEG (127 - 60)

#define MAVIC_CMD_SCALED_STEPS 127
#define MAVIC_CMD_SCALED_CENTER 127

#define SERIAL_HEADER 0xFF


#define FPLENGTH 15
static float flightPlan[FPLENGTH][3] = {{0.0f, 0.0f, 1.0f},
                               {-1.0f, -1.0f, 1.0f},
                               {-1.0f, 1.0f, 1.0f},
                               {1.0f, 1.0f, 1.0f},
                               {1.0f, -1.0f, 1.0f},
                               {-1.0f, -1.0f, 1.0f},
                               {-1.0f, 1.0f, 1.0f},
                               {1.0f, 1.0f, 1.0f},
                               {1.0f, -1.0f, 1.0f},
                               {-3.0f, -1.0f, 1.0f},
                               {-3.0f, 1.0f, 1.0f},
                               {3.0f, 1.0f, 1.0f},
                               {3.0f, -1.0f, 1.0f},
                               {-3.0f, -1.0f, 1.0f},
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
static velocity_t command;
static velocity_t vel;

uint8_t scaleCommandX(float unscaled)
{
  /*
  if (unscaled > 0.1f){
    return MAVIC_CMD_SCALED_POS;
  }
  else if (unscaled < -0.1f){
    return MAVIC_CMD_SCALED_NEG;
  }
  else{
    return 0;
  }
  */
  unscaled = fmaxf(-MAVIC_CMD_UNSCALED_MAX, fminf(MAVIC_CMD_UNSCALED_MAX, unscaled));

  float scaled = roundf((unscaled + 1.0f) * MAVIC_CMD_SCALED_STEPS); 
  return (uint8_t) (fmaxf(MAVIC_CMD_SCALED_CENTER-50, fminf(MAVIC_CMD_SCALED_CENTER+60, scaled)));
}

uint8_t scaleCommandY(float unscaled)
{
  unscaled = fmaxf(-MAVIC_CMD_UNSCALED_MAX, fminf(MAVIC_CMD_UNSCALED_MAX, unscaled));

  float scaled = roundf((unscaled + 1.0f) * MAVIC_CMD_SCALED_STEPS); 
  return (uint8_t) (fmaxf(MAVIC_CMD_SCALED_CENTER-60, fminf(MAVIC_CMD_SCALED_CENTER+50, scaled)));
}

uint8_t scaleCommand(float unscaled)
{
  unscaled = fmaxf(-MAVIC_CMD_UNSCALED_MAX, fminf(MAVIC_CMD_UNSCALED_MAX, unscaled));

  float scaled =  roundf((unscaled + 1.0f) * MAVIC_CMD_SCALED_STEPS); 
  return (uint8_t) (fmaxf(MAVIC_CMD_SCALED_NEG, fminf(MAVIC_CMD_SCALED_POS, scaled)));
}

void getRealVelocity(velocity_t* cmd, velocity_t* real)
{
  real->x = cmd->x == 0 ? 0 : ( cmd->x == MAVIC_CMD_SCALED_NEG ? VX_NEG : VX_POS );
  real->y = cmd->y == 0 ? 0 : ( cmd->y == MAVIC_CMD_SCALED_NEG ? VY_NEG : VY_POS );
  real->z = cmd->z == 0 ? 0 : ( cmd->z == MAVIC_CMD_SCALED_NEG ? VZ_NEG : VZ_POS );
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

  vTaskDelay(M2T(10000));
  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(50)); 
    
    // Get current multilaterated position
    currentPos.x = logGetFloat(posXid);
    currentPos.y = logGetFloat(posYid);
    currentPos.z = logGetFloat(posZid);

    // Calculate error to current goal
    delta.x = flightPlan[idx][0] - currentPos.x;
    delta.y = flightPlan[idx][1] - currentPos.y;
    delta.z = flightPlan[idx][2] - currentPos.z;
    
    if (fabsf(delta.x)<0.2f && fabsf(delta.y)<0.2f && idx<(FPLENGTH-1)){
      // Update current goal
      // Assemble the data
      packet.header = SERIAL_HEADER;
      packet.cmdZ = scaleCommand(0.0f);
      packet.cmdY = scaleCommandY(0.0f);
      packet.cmdX = scaleCommandX(0.0f);
      packet.cmdYaw = scaleCommand(0.0f);  
      
      // Send the three floats, byte by byte, to UART2
      uart2SendDataDmaBlocking(sizeof(ctrlPacket_t), (uint8_t *)(&packet));
      vTaskDelay(M2T(2000));
    
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
    packet.cmdY = scaleCommandY(command.y);
    packet.cmdX = scaleCommandX(command.x);
    packet.cmdYaw = scaleCommand(0.0f);   

    // Send the three floats, byte by byte, to UART2
    uart2SendDataDmaBlocking(sizeof(ctrlPacket_t), (uint8_t *)(&packet));

    getRealVelocity(&command, &vel);
    estimatorMheEnqueueVelocity(&vel);
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