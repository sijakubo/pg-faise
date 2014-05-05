/**
 * \file MemoryInterface.c
 * \brief	Memory Interface
 *
 * \author	Markus Lehmann & Jan-Gerd Meß
 * \date    04.02.2014
 */  
#include "MemoryInterface.h"
#include "defines/Defines.h"

volatile uint8_t writeReadLock; //Value to get the threads save
volatile uint8_t memory_interface_status;

uint8_t nearborder;

/**
 * \fn	void MemoryInterface_init()
 * \brief	Initialisiert das MemoryInterface und die benötigten Module
 *
 * \author	Markus Lehmann & Jan-Gerd Meß
 */
void MemoryInterface_init(){

	uint8_t PlatformAgentID_part1 = 0x00;
	uint8_t PlatformAgentID_part2 = 0x20;

	uint8_t spsID_part1 = 0xFF;
	uint8_t spsID_part2 = 0xFD;
	uint8_t d = 0;
	
	writeReadLock = FALSE;
	memory_interface_status = 0;

	/*WRITE SPS AND DEVICE ID */
//	spi_eeprom_bwrite( EEPROM_PROFIBUSADRESS, &profibus_adress );
//	spi_eeprom_bwrite( EEPROM_PROFIBUSADRESSSPS, &profibus_adress_sps );

	/*READ DEVICE ADDRESS FROM THE EEPROM*/
//	spi_eeprom_bread( EEPROM_PROFIBUSADRESS, &profibus_adress );
//
//	spi_eeprom_bread( EEPROM_CPAID_ID_PART_1, &PlatformAgentID_part1 );
//	spi_eeprom_bread( EEPROM_CPAID_ID_PART_2, &PlatformAgentID_part2 );
//
//	PlatformAgentID = PlatformAgentID_part1;
//	PlatformAgentID = PlatformAgentID << 8;
//	PlatformAgentID = PlatformAgentID | PlatformAgentID_part2;
//
//	spi_eeprom_bread( EEPROM_SPS_ID_PART_1, &spsID_part1 );
//	spi_eeprom_bread( EEPROM_SPS_ID_PART_2, &spsID_part2 );
//
//	sps_device_id = spsID_part1;
//	sps_device_id = sps_device_id << 8;
//	sps_device_id = sps_device_id | spsID_part2;

//	spi_eeprom_bread( EEPROM_NEARBORDER, &nearborder );
//	spi_eeprom_bread( EEPROM_COMMUNICATIONMODE, &communicationMode );	
	ExtflashInterface_init();
}

uint16_t MemoryInterface_getPlatformAgentID()
{
	return 0xF006;
}

//function to get the SPS-ID
uint16_t MemoryInterface_getSPSID()
{
	return 0xFFFD;
}

//function to get the nearborder value
uint8_t MemoryInterface_getNearborder(){
	return nearborder;
}

//function to get the communication mode
uint8_t MemoryInterface_getCommunicationMode(){
	return MODE_PROFIBUS;
}

//function to get the profibusadress
uint8_t MemoryInterface_getProfibusAdress(){
	return 0x30;//profibus_adress; //TODO
}


uint8_t MemoryInterface_getProfibusID(){
	return 0x06;//profibus_adress; //TODO
}


uint8_t MemoryInterface_writeComponentCell(uint8_t componentCount,uint16_t cell, uint8_t* data, uint8_t length){
	uint8_t result = FALSE;
	if(FALSE == writeReadLock){
		writeReadLock = TRUE;
		if(memory_interface_status != componentCount){
			ExtflashInterface_agent_restore(componentCount);
			memory_interface_status = componentCount;
		}
		result = ExtflashInterface_write(cell, data, length);
		writeReadLock = FALSE;
	}
	return result;
}

uint8_t MemoryInterface_readComponentCell(uint8_t componentCount, uint8_t cell, uint8_t* result, uint8_t length){
	uint8_t success = FALSE;
	if(FALSE == writeReadLock){
		writeReadLock = TRUE;
		if(memory_interface_status != componentCount){
			ExtflashInterface_agent_restore(componentCount);
			memory_interface_status = componentCount;
		}
		success = ExtflashInterface_read(cell, result, length);
		writeReadLock = FALSE;
	}
	return success;
}
