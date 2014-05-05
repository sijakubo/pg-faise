/**
 * \file uart_drv.c
 * \brief	Treiber für die UART0-Schnittstelle
 *
 * \author	Jan-Gerd Meß
 * \date    20.03.2014
 */ 

#include "drivers/uart_drv.h"
#include "lib/ringbuf.h"

/** Ringbuffer zum zeilenweisen Empfang */
volatile struct ringbuf rxbuf;

/** Speicher für den Ringbuffer */
volatile uint8_t rxbuf_data[BUFSIZE];

/**
 * \fn	void UartDriver_send(uint8_t *buf, uint8_t size)
 * \brief	Sendet byteweise Daten über die UART0-Schnittstelle
 *
 * \param buf Zeiger auf die Adresse der zu sendenden Daten
 * \param size Anzahl der zu sendenen Bytes
 *
 * \author	Jan-Gerd Meß
 */
void UartDriver_send(uint8_t *buf, uint8_t size)
{
	while(size > 0) {
		putchar(*buf++);
		size--;
	}
	putchar(UART0_LINE_END);
}

/**
 * \fn	int UartDriver_line_input_byte(unsigned char c)
 * \brief	Callback Funktion, die vom Interrupt aufgerufen wird, wenn ein Byte an UART0 empfangen wurde
 *
 * \param c Empfangenes Byte
 *
 * \author	Jan-Gerd Meß
 */
int UartDriver_line_input_byte(unsigned char c) 
{
  /** Overflow Indikator des Ringbuffers */
  static uint8_t overflow;

  // Liegt kein Overflow vor?
  if(!overflow) {
	// Speichere c im Ringbuffer
    if(ringbuf_put(&rxbuf, c) == 0) {
	  // Buffer voll, Byte wurde nicht gespeichert!
      overflow = 1;
    }
  } else {
	// Overflow! Zeichen bis zum Zeilenende ignorieren.
    if(c == UART0_LINE_END && ringbuf_put(&rxbuf, c) != 0) {
	  // Wieder Platz im Buffer? Overflow zurücksetzen!
      overflow = 0;
    }
  }

  process_poll(&UartDriver_recv_process);
  return 1;
}

/**
 * \brief	Verarbeitender Prozess der UART0-Schnittstelle
 *
 * Der Prozess wird angerufen (POLL), wenn ein Interrupt an der UART0-Schnittstelle ausgelöst wurde
 *
 * \author	Jan-Gerd Meß
 */
PROCESS(UartDriver_recv_process, "uart0 driver");
PROCESS_THREAD(UartDriver_recv_process, ev, data)
{
  //Zeilenbuffer
  static uint8_t buf[BUFSIZE];
  static int ptr = 0;

  PROCESS_BEGIN();

  serial_line_event_message = process_alloc_event();
  ptr = 0;

  while(1) {
	// Zeichen aus dem Ringbuffer holen
    int c = ringbuf_get(&rxbuf);
    
	// Ringbuffer leer? Warten bis zum nächsten Anruf
    if(c == -1) {
      PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_POLL);
    } else {
	  // Zeilenende noch nicht erreicht? Byte in Buffer speichern
      if(c != UART0_LINE_END) {
        if(ptr < BUFSIZE-1) {
          buf[ptr++] = (uint8_t)c;
        }
	  // Zeilenende erreicht? Buffer an Service übergeben und anschließend zurücksetzen
      } else {
        buf[ptr++] = 0x0a;

        com_receive(buf);

        ptr = 0;
      }
    }
  }

  PROCESS_END();
}

/**
 * \fn	void UartDriver_line_init(void)
 * \brief	Initialisierung des Treibers: Initialisierung des Ringbuffers und Start des verarbeitenden Prozesses
 *
 * \param c Empfangenes Byte
 *
 * \author	Jan-Gerd Meß
 */
void UartDriver_line_init(void)
{
  ringbuf_init(&rxbuf, rxbuf_data, sizeof(rxbuf_data));
  process_start(&UartDriver_recv_process, NULL);
}