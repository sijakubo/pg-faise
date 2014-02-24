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


#include <stdio.h>
#include "../../Atmel/Atmel Toolchain/AVR8 GCC/Native/3.4.2.1002/avr8-gnu-toolchain/avr/include/avr/iom128.h"
/*---------------------------------------------------------------------------*/
PROCESS(bolttest, "IO Test für Bolzen");
PROCESS(petest, "IO Test für Lichtschranken");
AUTOSTART_PROCESSES(&petest, &bolttest);
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(bolttest, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();

	while(1) {
		etimer_set(&et, CLOCK_SECOND / 100);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		bolt_release_and_separate_packet();
	}

	PROCESS_END();
}

PROCESS_THREAD(petest, ev, data)
{
	static struct etimer et;

	PROCESS_BEGIN();
	
	PORTC |= 0xF0;
	DDRC &= 0x0F & DDRC;
	
	leds_off(LEDS_GREEN | LEDS_RED);

	while(1) {
		etimer_set(&et, CLOCK_SECOND / 100);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		
		if(PINC & (1<<PC7)){
			leds_on(LEDS_GREEN);
		} else {
			leds_off(LEDS_GREEN);
		}
		
		if(PINC & (1<<PC5)){
			leds_on(LEDS_RED);
		} else {
			leds_off(LEDS_RED);
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/