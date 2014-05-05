/*
 * COMM.h
 *
 *  Created on: 27.05.2013
 *      Author: janp
 */

#ifndef MEMORYINTERFACE_H_
#define MEMORYINTERFACE_H_

#include "contiki.h"

void MemoryInterface_init(void);
uint16_t MemoryInterface_getPlatformAgentID(void);
uint16_t MemoryInterface_getDeviceID(void);
uint16_t MemoryInterface_getSPSID(void);
uint8_t MemoryInterface_getNearborder(void);
uint8_t MemoryInterface_getCommunicationMode(void);
uint8_t MemoryInterface_getProfibusAdress(void);
uint8_t MemoryInterface_getProfibusID(void);

uint8_t MemoryInterface_writeComponentCell(uint8_t componentCount,uint16_t cell, uint8_t* data, uint8_t length);
uint8_t MemoryInterface_readComponentCell(uint8_t componentCount, uint8_t cell, uint8_t* result, uint8_t length);

#endif /* COMM_H_ */
