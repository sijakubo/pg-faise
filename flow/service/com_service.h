/*
 * com_service.h
 *
 * Created: 05.04.2014 12:37:44
 *  Author: vmsieben
 */ 


#ifndef COM_SERVICE_H_
#define COM_SERVICE_H_

#include "contiki.h"
#include "sys/node-id.h"
#include "service/uart_service.h"

#define COM_CMD_RECEIVE_PACKAGE 0x50
#define COM_CMD_DELIVER_PACKAGE 0x55
#define COM_CMD_RET_COST 0x60

#define COM_CMD_DRIVE_IN_RAMP 0x01
#define COM_CMD_DRIVE_OUT_RAMP 0x02
#define COM_CMD_COST 0x10
#define COM_CMD_RECEIVE_PACKAGE_ACK 0x20
#define COM_CMD_DELIVER_PACKAGE_ACK 0x25

void com_receive(uint8_t* msg);


#endif /* COM_SERVICE_H_ */