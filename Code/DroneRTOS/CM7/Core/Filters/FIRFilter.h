//Code From: https://github.com/pms67

#ifndef FIR_FILTER_H
#define FIR_FILTER_H

#include <stdint.h>

#define FIR_FILTER_LENGTH 16//For original (1st and 3rd)
//#define FIR_FILTER_LENGTH 15 //For 2nd and 4th
//Circular buffer
typedef struct {
	float 	buf[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	float out;
} FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir, float inp);

#endif
