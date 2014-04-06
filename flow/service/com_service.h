/*
 * com_service.h
 *
 * Created: 05.04.2014 12:37:44
 *  Author: vmsieben
 */ 


#ifndef COM_SERVICE_H_
#define COM_SERVICE_H_

#include "contiki.h"
#include "sys/node-id.h"
#include "dev/leds.h"
#include "drivers/uart_drv.h"

#define COM_CMD_RECEIVE_PACKAGE 0x50
#define COM_CMD_DELIVER_PACKAGE 0x55
#define COM_CMD_RET_COST 0x11

#define COM_CMD_DRIVE_IN_RAMP 0x01
#define COM_CMD_DRIVE_OUT_RAMP 0x02
#define COM_CMD_COST 0x10
#define COM_CMD_RECEIVE_PACKAGE_ACK 0x20
#define COM_CMD_DELIVER_PACKAGE_ACK 0x25

//*********************************************
// Test-Commands for Ramps
//*********************************************
#define UART_LED_RED_COMMAND 0x61
#define UART_LED_GREEN_COMMAND 0x62
#define UART_RAMP_NUM_PACKAGES 0x63
#define UART_RAMP_RET_NUM_PACKAGES 0x64
#define UART_RAMP_BAY_STATUS 0x65
#define UART_RAMP_RET_BAY_STATUS 0x66
#define UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE 0x67
#define UART_RAMP_SEPARATE_PACKAGE 0x68
#define UART_RAMP_RELEASE_PACKAGE 0x69
//*********************************************

void com_receive(uint8_t* msg);
void com_send(uint8_t* msg, uint8_t len);


#endif /* COM_SERVICE_H_ */