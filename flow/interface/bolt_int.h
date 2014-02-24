/*
 * bolt_int.h
 *
 * Created: 24.02.2014 10:25:26
 *  Author: JanGerd
 */ 


#ifndef BOLT_INT_H_
#define BOLT_INT_H_


#include "sys/clock.h"

#define BOLT_SLIDE_TIME CLOCK_SECOND*3
#define BOLT_TIMEOUT CLOCK_SECOND

#define BOLT_LOWER BOLT_1
#define BOLT_UPPER BOLT_2

void bolt_init(void);
void bolt_release_and_separate_packet(void);

#endif /* BOLT_INT_H_ */