/*
 * uart_service.h
 *
 * Created: 10.03.2014 10:05:41
 *  Author: JanGerd
 */ 


#ifndef UART_SERVICE_H_
#define UART_SERVICE_H_

#include "drivers/uart_drv.h"

#define UART_LED_RED_COMMAND 0x01
#define UART_LED_GREEN_COMMAND 0x02
#define UART_LED_COMMAND_ARGS 1

//*********************************************
// Test-Commands for Ramps
//*********************************************
#define UART_RAMP_NUM_PACKAGES 0x03
#define UART_RAMP_RET_NUM_PACKAGES 0x04
#define UART_RAMP_BAY_STATUS 0x05
#define UART_RAMP__RET_BAY_STATUS 0x06
#define UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE 0x07
#define UART_RAMP_SEPARATE_PACKAGE 0x08
#define UART_RAMP_RELEASE_PACKAGE 0x09
//*********************************************

void uart0_send_command(uint8_t com, uint8_t* args);
void uart0_dispatch_message(uint8_t* msg);

#endif /* UART_SERVICE_H_ */