/*
 * uart_drv.c
 *
 * Created: 26.02.2014 13:01:47
 *  Author: JanGerd
 */ 
#include "drivers/uart_drv.h"

void uart0_send(uint8_t *buf, uint8_t size)
{
	while(size > 0) {
		putchar(*buf++);
		size--;
	}
}