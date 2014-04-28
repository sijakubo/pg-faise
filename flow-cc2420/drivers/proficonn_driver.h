/**
 * \file proficonn_drv.h
 * \brief	Treiber auf Pin-Ebene f�r den Proficon-Adapter
 *
 * \author	Jan-Gerd Me�
 * \date    14.04.2014
 */ 


#ifndef PROFICON_DRIVER_H_
#define PROFICON_DRIVER_H_

#include "contiki.h"
#include "contiki-conf.h"

/** Port f�r den SPI-Bus zum proficonn */
#define PROFICON_SPI_PORT PORTD

/** Data Direction Register f�r den SPI-Bus zum proficonn */
#define PROFICON_SPI_DDR DDRD

/** Eingangs-Register f�r den SPI-Bus zum proficonn */
#define PROFICON_SPI_IN PIND

/** Port f�r die Konfiguration des proficonn */
#define PROFICON_CONF_PORT PORTB

/** Data Direction Register f�r die Konfiguration des proficonn */
#define PROFICON_CONF_DDR DDRB

/** Eingangs-Register f�r die Konfigurations-Pins des proficonn */
#define PROFICON_SPI_IN PINB

/** Pin f�r den CLK des SPI-Bus zum proficonn */
#define PROFICON_CLK		PIND5

/** Pin f�r den SI / RXD des SPI-Bus zum proficonn */
#define PROFICON_RXD		PIND2

/** Pin f�r den SO / TXD des SPI-Bus zum proficonn */
#define PROFICON_TXD		PIND3

/** Pin f�r den Chipselect des proficonn */
#define PROFICON_CS			PINB6

/** Pin f�r den Chipselect des proficonn */
#define PROFICON_IRQ			PINB5

/** Pin f�r den Chipselect des proficonn */
#define PROFICON_RESET			PINB4


/** Byte, das zur �bertragung von Dont-Care-Werten genutzt wird*/
#define EXTFLASH_DONT_CARE 0xFF

void proficonn_init();

void proficonn_reset();

void proficonn_enable();
void proficonn_disable();

uint8_t proficonn_tr_byte(uint8_t spiOut);

uint8_t proficonn_is_irq(void);

#endif /* PROFICON_DRIVER_H_ */