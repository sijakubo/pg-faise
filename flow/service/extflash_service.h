/*
 * extflash_service.h
 *
 * Created: 26.03.2014 07:36:42
 *  Author: JanGerd
 */ 


#ifndef EXTFLASH_SERVICE_H_
#define EXTFLASH_SERVICE_H_

#include "drivers/extflash_drv.h"

void extflash_write_page_to_buffer1(uint16_t page);
void extflash_write_page_to_buffer2(uint16_t page);
void extflash_write_buffer1(uint16_t address, uint8_t len, uint8_t* data);
void extflash_write_buffer2(uint16_t address, uint8_t len, uint8_t* data);

uint8_t extflash_compare_buffer1_to_mem(uint16_t page);
uint8_t extflash_compare_buffer2_to_mem(uint16_t page);

void extflash_write_buffer1_to_mem(uint16_t page);
void extflash_write_buffer2_to_mem(uint16_t page);

void exflash_read_buffer1(uint16_t address, uint8_t* buffer, uint8_t len);
void extflash_read_buffer2(uint16_t address, uint8_t* buffer, uint8_t len);


#endif /* EXTFLASH_SERVICE_H_ */