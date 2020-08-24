/*
 *
 * serial_commands.h - send control commands through serial port 
 * 
 * Author: S. Pfeiffer, MAVLab TU Delft
 * 
 *  
 */

/* Send a velocity command to external controller via serial*/
void sendSerialVelocity(float vx, float vy);