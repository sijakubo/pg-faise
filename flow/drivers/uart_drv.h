/*
 * uart_drv.h
 *
 * Created: 26.02.2014 13:00:10
 *  Author: JanGerd
 */ 


#ifndef UART_DRV_H_
#define UART_DRV_H_

#ifndef BUFSIZE
#define BUFSIZE 16
#endif

/** Zeilenende für zeilenbasiert serielle Kommunikation */
#define UART0_LINE_END 0x0a

#include "contiki.h"
#include "contiki-conf.h"

extern process_event_t serial_line_event_message;

void uart0_send(uint8_t *buf, uint8_t size);

int uart0_line_input_byte(unsigned char c);

void uart0_line_init(void);

PROCESS_NAME(uart0_recv_process);

#endif /* UART_DRV_H_ */