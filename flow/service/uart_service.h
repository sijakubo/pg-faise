/*
 * uart_service.h
 *
 * Created: 10.03.2014 10:05:41
 *  Author: JanGerd
 */ 


#ifndef UART_SERVICE_H_
#define UART_SERVICE_H_

#include "drivers/uart_drv.h"

#define UART_COMMAND_DELEMITER 0x0A

#define UART_LED_RED_COMMAND 0x01
#define UART_LED_GREEN_COMMAND 0x02

void uart0_send_command(uint8_t com, uint8_t* args);

#endif /* UART_SERVICE_H_ */