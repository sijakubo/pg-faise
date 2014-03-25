/*
 * extflash_drv.h
 *
 * Created: 24.03.2014 16:43:24
 *  Author: JanGerd
 */ 


#ifndef EXTFLASH_DRV_H_
#define EXTFLASH_DRV_H_

#include "contiki.h"
#include "contiki-lib.h"
#include "dev/spi.h"
#include "contiki-conf.h"

#define EXTFLASH_BLOCKSIZE 264

#define EXTFLASH_OP_RDST 0x57

#define EXTFLASH_CS_PORT PORTA
#define EXTFLASH_CS_DDR DDRA
#define EXTFLASH_CS			PINA3

#define EXTFLASH_SPI_PORT PORTD
#define EXTFLASH_SPI_DDR DDRD
#define EXTFLASH_SPI_IN PIND
#define EXTFLASH_CLK		PIND5
#define EXTFLASH_RXD		PIND2
#define EXTFLASH_TXD		PIND3


void extflash_init();

uint8_t extflash_tr_byte(uint8_t spiOut, uint8_t isRead);

void extflash_write_page_to_buffer1(uint16_t page);
void extflash_write_page_to_buffer2(uint16_t page);
void extflash_write_buffer1(uint16_t address, uint8_t len, uint8_t* data);
void extflash_write_buffer2(uint16_t address, uint8_t len, uint8_t* data);

uint8_t extflash_compare_buffer1_to_mem(uint16_t page);
uint8_t extflash_compare_buffer2_to_mem(uint16_t page);

void extflash_write_buffer1_to_mem(uint16_t page);
void extflash_write_buffer2_to_mem(uint16_t page);

uint8_t read_status_register(void);

#endif /* EXTFLASH_DRV_H_ */