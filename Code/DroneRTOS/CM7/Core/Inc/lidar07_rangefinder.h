/*
 * RangeFinder.h
 *
 *  Created on: Mar 7, 2023
 *      Author: robin
 */

#ifndef INC_LIDAR07_RANGEFINDER_H_
#define INC_LIDAR07_RANGEFINDER_H_

#define WIDTH 32
//#include "stdint.h"
#include "main.h"
void RF_GetMeasurement(uint8_t* buffer);
void RF_SetFiltering(uint8_t* buffer);
void RF_SetSingleMode(uint8_t* buffer);
void RF_SetContMode(uint8_t* buffer);
void RF_SetSamplePeriod_100ms(uint8_t* buffer);
void RF_SetSamplePeriod_200ms(uint8_t* buffer);
void RF_ReadError(uint8_t* buffer);
uint32_t crcFast(uint8_t* message, uint8_t nBytes);
uint32_t crc32_fast(const uint8_t *s,uint8_t n) ;

#endif /* INC_LIDAR07_RANGEFINDER_H_ */
