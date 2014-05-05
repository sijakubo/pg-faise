/*
 * Defines.h
 *
 *  Created on: 10.06.2013
 *      Author: astasch
 */

#ifndef DEFINES_H_
#define DEFINES_H_


#define TRUE	1
#define FALSE 	0

// SPI configuration
#define BIT_CLK		(BIT4) // P8.4/UCB1CLK
#define BIT_SIMO	(BIT5) // P8.5/UCB1SIMO
#define BIT_SOMI	(BIT6) // P8.6/UCB1SOMI
#define BIT_CLKA	(BIT1) // P8.1/UCA1CLK
#define BIT_SIMOA	(BIT2) // P8.2/UCA1SIMO
#define BIT_SOMIA	(BIT3) // P8.3/UCA1SOMI

//defines for modes
#define MODE_PROFIBUS 1

//PROFIBUS_BUFFER_SIZE is the size of the last messages which have been received.
//The messages have to be saved, because otherwise the controller receives the
//message many times
#define PROFIBUS_BUFFER_SIZE 			1
#define PROFIBUS_QUEUE_SIZE				(8 * AGENTRTE_MAX_AGENTS)
#define PROFIBUS_MESSAGE_DATA_SIZE 		12 //= 16 - 4

#define MAX_MESSAGE_DATA_SIZE 			32
										//remember min header:
										//2 Byte = destination id
										//2 Byte = source id
										//2 Byte = 0x01
										//+2 Byte for parted Message
										//16 - 8 = 8 * n

#define MAX_MESSAGE_INOUT_DATA_SIZE (MAX_MESSAGE_DATA_SIZE + 2) //+2 because 0x01 and payloadlength
																//is in the data

//message ids and groups
#define ID_NO_ID						0x0000
#define ID_MIN_AGENT_ID					0x0001
#define ID_MAX_AGENT_ID					0xEFFF
#define ID_MIN_GROUP_ID					0xF000
#define ID_MAX_GROUP_ID					0xFFFD
#define ID_BROADCAST					0xFFFE
#define ID_INVALID						0xFFFF

//message modes
#define MESSAGE_MODE_PROFIBUS 			1 //Message from and to profibus

//Message Parameter
#define PARAMETER_DATA_BEGIN 			0x01

//FIPAParameter
#define PARAMETER_IntendedReceiver 		0x02
#define PARAMETER_ReplyTo 				0x03
#define PARAMETER_ReceivedObject 		0x04
#define PARAMETER_ConversationID 		0x05
#define PARAMETER_MessageID				0x06

//self made Parameter
#define PARAMETER_Acknowledge 			0x07
#define PARAMETER_Cancel				0x08
#define PARAMETER_RegisterAgent			0x09
#define PARAMETER_WhereIsID				0x10
#define PARAMETER_UpdatePhysical		0x11
#define PARAMETER_HereIs				0x12
#define PARAMETER_UpdatePosition		0x13
#define PARAMETER_Request				0x14
#define PARAMETER_MessagePart			0x15
											//A message with this parameter updates
											//lightbarries and conveyorbelts

//defines update physical to register agents or groups, unregister agents and setSpeed
#define UpdatePhysical_setSpeed			0x01
#define UpdatePhysical_agentRegister	0x02
#define UpdatePhysical_agentUnregister	0x03

//defines for the Agents
#define MESSAGE_ID_NOT_SET				0x00//NO SUCCESS also
#define MESSAGE_ID_SUCCESS				0x01
#define MESSAGE_ID_FIRST				0x02

#define MESSAGE_TYPE_NORMAL				0x0000
#define MESSAGE_TYPE_REPLYTO			0x0001
#define MESSAGE_TYPE_RECEIVEDOBJECT		0x0002
#define MESSAGE_TYPE_CONVERSATION		0x0004
#define MESSAGE_TYPE_ACKNOWLEDGE		0x0008
#define MESSAGE_TYPE_CANCEL				0x0010
#define MESSAGE_TYPE_REQUEST			0x0020
#define MESSAGE_TYPE_REGISTERREQUEST	0x0040
#define MESSAGE_TYPE_UPDATEPHYSICAL		0x0080
#define MESSAGE_TYPE_MESSAGEID			0x0100

#define MESSAGE_PART_POSITION			0x00
#define MESSAGE_PART_BUFFER_SIZE		(8 * AGENTRTE_MAX_AGENTS)
										//should be a ratio of PROFIBUS_MESSAGE_DATA_SIZE,
										//MAX_MESSAGE_DATA_SIZE and AGENTRTE_MAX_AGENTS
										//in the current setup a message can be split in
										//two messages

#define AGENTRTE_MAX_AGENTS				7
#define AGENTRTE_QUEUE_SIZE				(2 * AGENTRTE_MAX_AGENTS)

#define CMCONTROL_MAX_AGENTS			AGENTRTE_MAX_AGENTS

//prioritys for sending messages
#define PRIORITY_NO_MESSAGE				-1
#define PRIORITY_AGENT 					1
#define PRIORITY_DEVICE					2
#define PRIORITY_SENSOR 				3
#define PRIORITY_CONVEYORBELT 			4

//lightbarrier defines
#define LIGHTBARRIER_COUNT				2

#define LIGHTBARRIER_IN					0 //the front barrier
#define LIGHTBARRIER_OUT				1 //the barrier at the back
#define LIGHTBARRIER_LEFT				2
#define LIGHTBARRIER_RIGHT				3

#define LIGHTBARRIER_STATE_OFF			1
#define LIGHTBARRIER_STATE_ON			1

//Max space one component has in the EEPROM
#define EEPROM_WRITEINGQUEUESIZE		3
#define EEPROM_MAXCOMPONENTSPACE		20 // 20 byte pro agent / component
#define EEPROM_EMPTYQUEUEADRESS		((uint32_t)0x00) // to show that this
													 //queue-element is empty
#define EEPROM_COMPONENT_BEGIN		((uint32_t)0x10) //adress of the first cell

//EEPROM adresses and range
#define EEPROM_SPS_ID_PART_1		((uint32_t)0x01)
#define EEPROM_SPS_ID_PART_2		((uint32_t)0x02)
#define EEPROM_CPAID_ID_PART_1		((uint32_t)0x03)	//conveyorPlatformAgentID part 1
#define EEPROM_CPAID_ID_PART_2		((uint32_t)0x04)	//conveyorPlatformAgentID part 2
#define EEPROM_NEARBORDER			((uint32_t)0x05)
#define EEPROM_COMMUNICATIONMODE	((uint32_t)0x06)
#define EEPROM_PROFIBUSADRESS		((uint32_t)0x07)

//message types of the SPS
#define SPS_MESSAGE_UPDATEPHYSICAL		1
#define SPS_MESSAGE_REGISTERID			2
#define SPS_MESSAGE_UNREGISTERID		3

//agenttypes for registration of agents
#define AGENTTYPE_INVALID				0
#define AGENTTYPE_PLATFORMAGENT			1
#define AGENTTYPE_ROUTINGAGENT			2
#define AGENTTYPE_ORDERAGENT			3
#define AGENTTYPE_PACKAGEAGENT			4

#endif /* DEFINES_H_ */
