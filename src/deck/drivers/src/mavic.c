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

// Position Controller
#define CONTROL_UPDATE_RATE RATE_25_HZ 
#define CONTROL_UPDATE_DT (1.0/CONTROL_UPDATE_RATE)
#define VELOCITY_P_GAIN 270.0f//127.0f
#define VELOCITY_D_GAIN 340.0f
#define VELOCITY_I_GAIN 0.0f

#define SETPOINT_ACCURACY 0.2f // How close to the setpoint we go to the next
#define I_GAIN_THRESHOLD (SETPOINT_ACCURACY+0.2f)  // How close to the setpoint do we add I gain

// Serial Communication
#define SERIAL_HEADER 0xFF

/* Command Scaling:
 * Commands to the Mavic Transmitter are PWM signals between 0 and 255,
 * where 127 is a stop command on that particular axis. Due to 
 * quality constraints on the images, it makes sense to limit the values
 * Every Axis (Thrust, Roll, Pitch, Yaw) seems to also have a deadband around 0. 
 * Unfortunately, both the deadbands and the limits are not fully symmetric 
 */

#define MAVIC_CMD_SCALED_CENTER 127

#define THRUST_LIMIT_LOW      (MAVIC_CMD_SCALED_CENTER-60)
#define THRUST_DEADBAND_LOW   (MAVIC_CMD_SCALED_CENTER-10)
#define THRUST_DEADBAND_HIGH  (MAVIC_CMD_SCALED_CENTER+10)
#define THRUST_LIMIT_HIGH     (MAVIC_CMD_SCALED_CENTER+60)

#define ROLL_LIMIT_LOW      (MAVIC_CMD_SCALED_CENTER-60)
#define ROLL_DEADBAND_LOW   (MAVIC_CMD_SCALED_CENTER-10)
#define ROLL_DEADBAND_HIGH  (MAVIC_CMD_SCALED_CENTER+10)
#define ROLL_LIMIT_HIGH     (MAVIC_CMD_SCALED_CENTER+50)

#define PITCH_LIMIT_LOW      (MAVIC_CMD_SCALED_CENTER-50)
#define PITCH_DEADBAND_LOW   (MAVIC_CMD_SCALED_CENTER-10)
#define PITCH_DEADBAND_HIGH  (MAVIC_CMD_SCALED_CENTER+10)
#define PITCH_LIMIT_HIGH     (MAVIC_CMD_SCALED_CENTER+60)

#define YAW_LIMIT_LOW      (MAVIC_CMD_SCALED_CENTER-60)
#define YAW_DEADBAND_LOW   (MAVIC_CMD_SCALED_CENTER-10)
#define YAW_DEADBAND_HIGH  (MAVIC_CMD_SCALED_CENTER+10)
#define YAW_LIMIT_HIGH     (MAVIC_CMD_SCALED_CENTER+60)

#define MAVIC_CMD_UNSCALED_MAX 1.0f

#define MAVIC_CMD_SCALED_STEPS 127

// Flightplan
#define FPLENGTH 1
static float flightPlan[FPLENGTH][3] = {{-2.0f, -2.0f, 1.0f}};

/*
#define FPLENGTH 18
static float flightPlan[FPLENGTH][3] = {
                                {-2.0f, -2.0f, 1.0f},
                                { 2.0f, -2.0f, 1.0f},
                                { 2.0f, -1.5f, 1.0f},
                                {-2.0f, -1.5f, 1.0f},
                                {-2.0f, -1.0f, 1.0f},
                                { 2.0f, -1.0f, 1.0f},
                                { 2.0f, -0.5f, 1.0f},
                                {-2.0f, -0.5f, 1.0f},
                                {-2.0f, 0.0f, 1.0f},
                                { 2.0f, 0.0f, 1.0f},
                                { 2.0f, 0.5f, 1.0f},
                                {-2.0f, 0.5f, 1.0f},
                                {-2.0f, 1.0f, 1.0f},
                                { 2.0f, 1.0f, 1.0f},
                                { 2.0f, 1.5f, 1.0f},
                                {-2.0f, 1.5f, 1.0f},
                                {-2.0f, 2.0f, 1.0f},
                                { 2.0f, 2.0f, 1.0f}};
*/
typedef struct ctrlPacket_s {
  uint8_t header;
  uint8_t cmdZ;
  uint8_t cmdY;
  uint8_t cmdX;
  uint8_t cmdYaw;
} ctrlPacket_t;

typedef struct mavicControlAxis_s{
  uint8_t lim_l;
  uint8_t db_l;
  uint8_t db_h;
  uint8_t lim_h;
} mavicControlAxis_t;

static const mavicControlAxis_t axis_x = {.lim_l = PITCH_LIMIT_LOW, .db_l = PITCH_DEADBAND_LOW,
                                          .db_h = PITCH_DEADBAND_HIGH, .lim_h = PITCH_LIMIT_HIGH };
static const mavicControlAxis_t axis_y = {.lim_l = ROLL_LIMIT_LOW, .db_l = ROLL_DEADBAND_LOW,
                                          .db_h = ROLL_DEADBAND_HIGH, .lim_h = ROLL_LIMIT_HIGH };
static const mavicControlAxis_t axis_z = {.lim_l = THRUST_LIMIT_LOW, .db_l = THRUST_DEADBAND_LOW,
                                          .db_h = THRUST_DEADBAND_HIGH, .lim_h = THRUST_LIMIT_HIGH };
static const mavicControlAxis_t axis_yaw = {.lim_l = YAW_LIMIT_LOW, .db_l = YAW_DEADBAND_LOW,
                                          .db_h = YAW_DEADBAND_HIGH, .lim_h = YAW_LIMIT_HIGH };

static bool isInit;

static float uwb_filter_queue[UWB_FILTER_LENGTH][3] = {{0.0f, 0.0f, 0.0f}};
static point_t currentPos;
static velocity_t command;
//static velocity_t vel;

// Remove deadband and enforce upper and lower limits on command
uint8_t scaleCommand(float unscaled, const mavicControlAxis_t* axis)
{
  if (unscaled > 0){
    return (uint8_t) fminf(unscaled + axis->db_h, axis->lim_h);
  }
  else if (unscaled < 0){
    return (uint8_t) fmaxf(unscaled + axis->db_l, axis->lim_l);
  }
  else {    // unscaled == 0
    return (uint8_t) MAVIC_CMD_SCALED_CENTER;
  }
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

  vector_t error = {.x=0.0, .y=0.0, .z=0.0};
  vector_t last_error = {.x=0.0, .y=0.0, .z=0.0};
  vector_t d_error = {.x=0.0, .y=0.0, .z=0.0};
  vector_t I_error = {.x=0.0, .y=0.0, .z=0.0};

  TickType_t lastWakeTime = xTaskGetTickCount();

  int idx = 0;

  vTaskDelay(M2T(10000));
  while (1)
  {
    // Set the loop unlock time in ms
    vTaskDelayUntil(&lastWakeTime, M2T(1000*CONTROL_UPDATE_DT)); 
    
    // Get current multilaterated position
    currentPos.x = logGetFloat(posXid);
    currentPos.y = logGetFloat(posYid);
    currentPos.z = logGetFloat(posZid);

    // Keep track of the previous error for derivative gain
    last_error.x = error.x;
    last_error.y = error.y;
    last_error.z = error.z;

    // Calculate error to current goal
    error.x = flightPlan[idx][0] - currentPos.x;
    error.y = flightPlan[idx][1] - currentPos.y;
    error.z = flightPlan[idx][2] - currentPos.z;
    
    if (false){ //(fabsf(error.x)<SETPOINT_ACCURACY && fabsf(error.y)<SETPOINT_ACCURACY){
      // Stop for a bit
      packet.header = SERIAL_HEADER;
      packet.cmdZ = scaleCommand(0.0f, &axis_z);
      packet.cmdY = scaleCommand(0.0f, &axis_y);
      packet.cmdX = scaleCommand(0.0f, &axis_x);
      packet.cmdYaw = scaleCommand(0.0f, &axis_yaw);  
      uart2SendDataDmaBlocking(sizeof(ctrlPacket_t), (uint8_t *)(&packet));
      vTaskDelay(M2T(2000));
      
      // Update current goal and errors
      if (idx<(FPLENGTH-1)){
        idx++;
        error.x = flightPlan[idx][0] - currentPos.x;
        error.y = flightPlan[idx][1] - currentPos.y;
        error.z = flightPlan[idx][2] - currentPos.z;
        last_error.x = 0.0;
        last_error.y = 0.0;
        last_error.z = 0.0;
        I_error.x = 0.0;
        I_error.y = 0.0;
        I_error.z = 0.0;
      }
      else{
        while (1)
        {
          vTaskDelay(M2T(10000));
          uart2SendDataDmaBlocking(sizeof(ctrlPacket_t), (uint8_t *)(&packet));
        }
      }
    }

    // Error difference
    d_error.x = error.x - last_error.x;
    d_error.y = error.y - last_error.y;
    d_error.z = error.z - last_error.z;

    // Accumulate Error if error becomes small
    if (fabsf(error.x)<I_GAIN_THRESHOLD && fabsf(error.y)<I_GAIN_THRESHOLD){
      I_error.x += error.x;
      I_error.y += error.y;
      I_error.z += error.z;
    }
    // Velocity command PD-controller
    command.x = VELOCITY_P_GAIN*error.x + VELOCITY_D_GAIN*d_error.x + VELOCITY_I_GAIN*I_error.x;
    command.y = VELOCITY_P_GAIN*error.y + VELOCITY_D_GAIN*d_error.y + VELOCITY_I_GAIN*I_error.y;
    command.z = 0;
    

    // Assemble the data
    packet.header = SERIAL_HEADER;
    packet.cmdZ = scaleCommand(command.z, &axis_z);
    packet.cmdY = scaleCommand(command.y, &axis_y);
    packet.cmdX = scaleCommand(command.x, &axis_x);
    packet.cmdYaw = scaleCommand(0.0f, &axis_yaw);   

    // Send the three floats, byte by byte, to UART2
    DEBUG_PRINT("x=%d, y=%d, z=%d, yaw=%d\n", packet.cmdX, packet.cmdY, packet.cmdZ, packet.cmdYaw);
    uart2SendDataDmaBlocking(sizeof(ctrlPacket_t), (uint8_t *)(&packet));

    //getRealVelocity(&command, &vel);
    //estimatorMheEnqueueVelocity(&vel);
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