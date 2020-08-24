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
#include <string.h>

// xbee connected to UART2 (PA2/PA3)
/*
#define GPIO_PORT GPIOA
#define GPIO_PIN_RX GPIO_Pin_3
#define GPIO_PIN_TX GPIO_Pin_2
*/

typedef struct serialCommandPacket_s{
    uint8_t msg[12];
}serialCommandPacket_t;


void sendSerialVelocity(float vx, float vy)
{   
    DEBUG_PRINT("Sending Serial Velocity\n");
    //serialCommandPacket_t tx_packet;
    //char message[12] = "Test Message";
    //memcpy(tx_packet.msg, message, 12);
    //uart2SendDataDmaBlocking(sizeof(serialCommandPacket_t), (uint8_t*) &tx_packet);
    uart2SendDataDmaBlocking(36, (uint8_t *)"Testing UART2 DMA and it is working\n");
    uart2SendData(1, (uint8_t *)0x59);
}