/*
 *
 * serial_commands.c - send control commands through serial port 
 * 
 * Author: S. Pfeiffer, MAVLab TU Delft
 * 
 *  
 */

#include "stm32f4xx.h"
#include "uart2.h"
#include "debug.h"
// #include <string.h>

// xbee connected to UART2 (PA2/PA3)
/*
#define GPIO_PORT GPIOA
#define GPIO_PIN_RX GPIO_Pin_3
#define GPIO_PIN_TX GPIO_Pin_2
*/

#define SERIAL_COMMAND_HEADER 0x00

typedef struct serialCommandPacket_s{
    uint8_t header;
    int vx;
    int vy;
}serialCommandPacket_t;


void sendSerialVelocity(float vx, float vy)
{   
    DEBUG_PRINT("Sending Serial Velocity\n");
    serialCommandPacket_t tx_packet;
    tx_packet.header = SERIAL_COMMAND_HEADER;
    tx_packet.vx = vx;
    tx_packet.vy = vy;
    uart2SendDataDmaBlocking(sizeof(serialCommandPacket_t), (uint8_t*) &tx_packet);
}