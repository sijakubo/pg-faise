/**
 * \file photosensor_int.h
 * \brief	Interface f�r die Lichtschranken
 *
 * \author	Jan-Gerd Me�
 * \date    24.02.2014
 */ 

#ifndef PHOTOSENSOR_INT_H_
#define PHOTOSENSOR_INT_H_


void PhotosensorInterface_init(void);
uint8_t PhotosensorInterface_is_bay_occupied(uint8_t i);
uint8_t PhotosensorInterface_num_packages(void);


#endif /* PHOTOSENSOR_INT_H_ */