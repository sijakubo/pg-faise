/*
 * uart_drv.c
 *
 * Created: 26.02.2014 13:01:47
 *  Author: JanGerd
 */ 
#include "drivers/uart_drv.h"
#include "lib/ringbuf.h"

static struct ringbuf rxbuf;
static uint8_t rxbuf_data[BUFSIZE];

void uart0_send(uint8_t *buf, uint8_t size)
{
	while(size > 0) {
		putchar(*buf++);
		size--;
	}
}

int uart0_line_input_byte(unsigned char c) 
{
  static uint8_t overflow;

  if(!overflow) {
    if(ringbuf_put(&rxbuf, c) == 0) {
      overflow = 1;
    }
  } else {
    if(c == UART0_LINE_END && ringbuf_put(&rxbuf, c) != 0) {
      overflow = 0;
    }
  }

  process_poll(&uart0_recv_process);
  return 1;
}

PROCESS(uart0_recv_process, "uart0 driver");
PROCESS_THREAD(uart0_recv_process, ev, data)
{
  static uint8_t buf[BUFSIZE];
  static int ptr = 0;

  PROCESS_BEGIN();

  serial_line_event_message = process_alloc_event();
  ptr = 0;

  while(1) {
    int c = ringbuf_get(&rxbuf);
    
    if(c == -1) {
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
    } else {
      if(c != UART0_LINE_END) {
        if(ptr < BUFSIZE-1) {
          buf[ptr++] = (uint8_t)c;
        }
      } else {
        buf[ptr++] = 0x0a;

        uart0_dispatch_message(buf);

        ptr = 0;
      }
    }
  }

  PROCESS_END();
}

void uart0_line_init(void)
{
  ringbuf_init(&rxbuf, rxbuf_data, sizeof(rxbuf_data));
  process_start(&uart0_recv_process, NULL);
}