#include <stdio.h>
#include <avr/pgmspace.h>

#include "dev/leds.h"
#include "dev/rs232.h"
#include "dev/ds2401.h"
#include "sys/node-id.h"

#include "drivers/uart_drv.h"

/*---------------------------------------------------------------------------*/
/*void
init_radio(void)
{
	cc2420_init();
	uint8_t power = cc2420_get_txpower();
	printf("%u\n",power);
	power = 3;
	printf("%u\n",power);
	cc2420_set_txpower(power);
	power = cc2420_get_txpower();
	printf("%u\n",power);
}*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
void
init_usart(void)
{
  /* First rs232 port for debugging */
  rs232_init(RS232_PORT_0, USART_BAUD_115200,
             USART_PARITY_NONE | USART_STOP_BITS_1 | USART_DATA_BITS_8);

#if WITH_UIP || WITH_UIP6
 // slip_arch_init(USART_BAUD_115200);
    rs232_redirect_stdout(RS232_PORT_0);
#else
    rs232_redirect_stdout(RS232_PORT_0);
#endif /* WITH_UIP */

}
/*---------------------------------------------------------------------------*/
int
main(void)
{
  leds_init();
  leds_on(LEDS_RED);
  extflash_init();
  /* Initialize USART */
  init_usart();
  /* Clock */
  clock_init();
  leds_on(LEDS_GREEN);
  ds2401_init();
  node_id_restore();
  random_init(ds2401_id[0] + node_id);
  rtimer_init();
  /* Process subsystem */
  process_init();
  process_start(&etimer_process, NULL);
  ctimer_init();
  leds_on(LEDS_YELLOW);
  init_net();
  printf_P(PSTR(CONTIKI_VERSION_STRING " started. Node id %u\n"), node_id);
  leds_off(LEDS_ALL);
  bolt_init();
  radio_init_drv();
  radio_init();
  photosensor_init();
  /* Autostart processes */
  autostart_start(autostart_processes);
  uart0_line_init();
  rs232_set_input(RS232_PORT_0, uart0_line_input_byte) ;
  /* Main scheduler loop */
  do {
    process_run();
  }while(1);
  return 0;
}
