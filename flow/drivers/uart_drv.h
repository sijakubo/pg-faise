/*
 * uart_drv.h
 *
 * Created: 26.02.2014 13:00:10
 *  Author: JanGerd
 */ 


#ifndef UART_DRV_H_
#define UART_DRV_H_

#define IS_INDEXABLE(arg) (sizeof(arg[0]))
#define IS_ARRAY(arg) (IS_INDEXABLE(arg) && (((void *) &arg) == ((void *) arg)))
#define ARRAYSIZE(arr) (IS_ARRAY(arr) ? (sizeof(arr) / sizeof(arr[0])) : 0)

#include "contiki-conf.h"

void usart0_print(uint8_t *buf, uint8_t size);

#endif /* UART_DRV_H_ */