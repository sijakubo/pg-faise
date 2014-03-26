/**
 * \file extflash_drv.h
 * \brief	Treiber auf Pin-Ebene f�r Zugriffe auf den externen Speicher
 *
 * \author	Jan-Gerd Me�
 * \date    24.03.2014
 */


#ifndef EXTFLASH_DRV_H_
#define EXTFLASH_DRV_H_

#include "contiki.h"
#include "contiki-lib.h"
#include "dev/spi.h"
#include "contiki-conf.h"

/** Port f�r den SPI-Bus zum Speicher*/
#define EXTFLASH_SPI_PORT PORTD

/** Data Direction Register f�r den SPI-Bus zum Speicher */
#define EXTFLASH_SPI_DDR DDRD

/** Eingangs-Pin f�r den SPI-Bus zum Speicher */
#define EXTFLASH_SPI_IN PIND

/** Port f�r den Chipselect des Speichers*/
#define EXTFLASH_CS_PORT PORTA

/** Data Direction Register f�r den Chipselect des Speichers*/
#define EXTFLASH_CS_DDR DDRA


/** Pin f�r den CLK des SPI-Bus zum Speicher */
#define EXTFLASH_CLK		PIND5

/** Pin f�r den SI / RXD des SPI-Bus zum Speicher */
#define EXTFLASH_RXD		PIND2

/** Pin f�r den SO / TXD des SPI-Bus zum Speicher */
#define EXTFLASH_TXD		PIND3

/** Pin f�r den Chipselect des Speichers */
#define EXTFLASH_CS			PINA3

/** Gr��e einer Seite (Page) im Speicher */
#define EXTFLASH_BLOCKSIZE 264

/** Anzahl der Pages im Speicher */
#define EXTFLASH_NUM_PAGES 2048

/** Compare-Bit im Statusregister des Speichers */
#define EXTFLASH_ST_COMPARE 6

/** Opcode Statusregister */
#define EXTFLASH_OP_RDST 0x57

/** Byte, das zur �bertragung von Dont-Care-Werten genutzt wird*/
#define EXTFLASH_DONT_CARE 0xFF



void extflash_init();

uint8_t extflash_tr_byte(uint8_t spiOut, uint8_t isRead);

uint8_t extflash_read_status_register(void);

uint8_t extflash_get_last_compare(void);

#endif /* EXTFLASH_DRV_H_ */