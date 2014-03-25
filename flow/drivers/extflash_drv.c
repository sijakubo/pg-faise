/*
 * extflash_drv.c
 *
 * Created: 24.03.2014 16:43:13
 *  Author: JanGerd
 */ 

#include "drivers/extflash_drv.h"

void extflash_init(){
	EXTFLASH_SPI_DDR |= 1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD;
	EXTFLASH_SPI_DDR &= ~1<<EXTFLASH_RXD;
	
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK) | 1<<EXTFLASH_TXD;
	
	EXTFLASH_CS_PORT |= 1<<EXTFLASH_CS;
	EXTFLASH_CS_DDR |= 1<<EXTFLASH_CS;
}

void extflash_enable(){
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD);
	EXTFLASH_CS_PORT &= ~(1<<EXTFLASH_CS);
}

void extflash_disable(){
	EXTFLASH_CS_PORT |= 1<<EXTFLASH_CS;
	EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD);
}

uint8_t extflash_tr_byte(uint8_t spiOut){
	uint8_t recv = 0;
	uint8_t i = 8;

	printf("SPI SENT: %u\n", spiOut);
	for(;i >= 1; i--){
		EXTFLASH_SPI_PORT &= ~(1<<EXTFLASH_CLK | 1<<EXTFLASH_TXD);
		
		if(spiOut & (1<<(i-1))){
			EXTFLASH_SPI_PORT |= 1<<EXTFLASH_TXD;
		}
		
		EXTFLASH_SPI_PORT |= 1<<EXTFLASH_CLK;
		
		if(PIND & (1<<EXTFLASH_RXD)){
			recv |= 1<<(i-1);
		}
	}
	printf("RECEIVED: %u\n\n", recv);
	return recv;
}

void extflash_write_page_to_buffer1(uint16_t page){
	
}

void extflash_write_page_to_buffer2(uint16_t page){
	
}

void extflash_write_buffer1(uint16_t address, uint8_t len, uint8_t* data){
	
}

void extflash_write_buffer2(uint16_t address, uint8_t len, uint8_t* data){
	
}

uint8_t extflash_compare_buffer1_to_mem(uint16_t page){
	return 0;
}

uint8_t extflash_compare_buffer2_to_mem(uint16_t page){
	return 0;
}

void extflash_write_buffer1_to_mem(uint16_t page){
	
}

void extflash_write_buffer2_to_mem(uint16_t page){
	
}

static uint8_t read_status_register(void)
{
}