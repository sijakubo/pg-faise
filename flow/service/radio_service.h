/*
 * radio_service.h
 *
 * Created: 28.04.2014 10:25:46
 *  Author: Malte
 */ 


#ifndef RADIO_SERVICE_H_
#define RADIO_SERVICE_H_



#include "contiki.h"
#include "net/rime.h"
#include "random.h"
#include "dev/button-sensor.h"
#include "dev/leds.h"
#include <stdio.h>

/************************************************************************/
/* Test Commands                                                        */
/************************************************************************/
#define UART_LED_RED_COMMAND 0x61
#define UART_LED_GREEN_COMMAND 0x62
#define UART_RAMP_NUM_PACKAGES 0x63
#define UART_RAMP_RET_NUM_PACKAGES 0x64
#define UART_RAMP_BAY_STATUS 0x65
#define UART_RAMP_RET_BAY_STATUS 0x66
#define UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE 0x67
#define UART_RAMP_SEPARATE_PACKAGE 0x68
#define UART_RAMP_RELEASE_PACKAGE 0x69
/************************************************************************/

void openabcchannel(void);
void sendmessage(uint8_t* msgtext, uint8_t length);

#endif /* RADIO_SERVICE_H_ */