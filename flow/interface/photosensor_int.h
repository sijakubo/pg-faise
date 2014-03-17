/*
 * photosensor_int.h
 *
 * Created: 24.02.2014 16:43:59
 *  Author: JanGerd
 */ 


#ifndef PHOTOSENSOR_INT_H_
#define PHOTOSENSOR_INT_H_


void photosensor_init(void);
uint8_t photosensor_is_bay_occupied(uint8_t i);
uint8_t photosensor_num_packages(void);


#endif /* PHOTOSENSOR_INT_H_ */