/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Testing the multihop forwarding layer (multihop) in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 *
 *
 *         This example shows how to use the multihop Rime module, how
 *         to use the announcement mechanism, how to manage a list
 *         with the list module, and how to allocate memory with the
 *         memb module.
 *
 *         The multihop module provides hooks for forwarding packets
 *         in a multi-hop fashion, but does not implement any routing
 *         protocol. A routing mechanism must be provided by the
 *         application or protocol running on top of the multihop
 *         module. In this case, this example program provides the
 *         routing mechanism.
 *
 *         The routing mechanism implemented by this example program
 *         is very simple: it forwards every incoming packet to a
 *         random neighbor. The program maintains a list of neighbors,
 *         which it populated through the use of the announcement
 *         mechanism.
 *
 *         The neighbor list is populated by incoming announcements
 *         from neighbors. The program maintains a list of neighbors,
 *         where each entry is allocated from a MEMB() (memory block
 *         pool). Each neighbor has a timeout so that they do not
 *         occupy their list entry for too long.
 *
 *         When a packet arrives to the node, the function forward()
 *         is called by the multihop layer. This function picks a
 *         random neighbor to send the packet to. The packet is
 *         forwarded by every node in the network until it reaches its
 *         final destination (or is discarded in transit due to a
 *         transmission error or a collision).
 *
 */

 
#include "contiki.h"
#include "net/rime.h"
#include "lib/list.h"
#include "lib/memb.h"
#include "lib/random.h"
#include "dev/leds.h"
#include "interface/bolt_int.h"
#include "interface/photosensor_int.h"
#include "service/uart_service.h"

/*---------------------------------------------------------------------------*/
PROCESS(ramptest, "Test for Ramps");
AUTOSTART_PROCESSES(&ramptest);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

struct broadcast_conn broadcast;

static void broadcast_recv(struct broadcast_conn *c, const rimeaddr_t *from)
{
	uint8_t* msg = (uint8_t*)packetbuf_dataptr();
	if(msg[0] == UART_RAMP_RELEASE_PACKAGE){
		leds_toggle(LEDS_GREEN);
		bolt_release();
	} else if(msg[0] == UART_RAMP_SEPARATE_PACKAGE){
		leds_toggle(LEDS_RED);
		bolt_separate();
	} else if(msg[0] == UART_RAMP_RELEASE_AND_SEPARATE_PACKAGE){
		leds_toggle(LEDS_RED | LEDS_GREEN);
		bolt_release_and_separate();
	} else if(msg[0] == UART_RAMP_NUM_PACKAGES){
		uint8_t newmsg[2];
		newmsg[0] = UART_RAMP_RET_NUM_PACKAGES;
		newmsg[1] = photosensor_num_packages();
		packetbuf_copyfrom(newmsg, 2);
		broadcast_send(&broadcast);
	} else if(msg[0] == UART_RAMP_RET_NUM_PACKAGES){
		printf("Number of packages: %u\n", msg[1]);
	} 
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

PROCESS_THREAD(ramptest, ev, data)
{
	static struct etimer et;
	PROCESS_BEGIN();
	
	leds_off(LEDS_ALL);
	
	broadcast_open(&broadcast, 129, &broadcast_call);
	
	while(1) {
		leds_toggle(LEDS_YELLOW);
		etimer_set(&et, CLOCK_SECOND);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	}

	PROCESS_END();
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/