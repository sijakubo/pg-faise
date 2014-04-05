/*
 * extflash_service.c
 *
 * Created: 26.03.2014 07:36:13
 *  Author: JanGerd
 */ 

#include "extflash_service.h"

void extflash_write_page_to_buffer1(uint16_t page){
	sreg = SREG;
	cli();
	
	extflash_enable();
	
	exflash_disable();
	
	SREG = sreg;
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