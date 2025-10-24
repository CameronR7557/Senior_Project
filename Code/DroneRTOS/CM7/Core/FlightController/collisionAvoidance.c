/*
 * collisionAvoidance.c
 *
 *  Created on: Feb 6, 2024
 *      Author: Cameron Robinson
 */
#include <stdio.h>
#include "../FlightController/collisionAvoidance.h"
extern DMA_HandleTypeDef hdma_memtomem_dma1_stream5;

//Need to track yaw, and velocity of drone. The angle indicies need to adjust based on yaw. Make something like a hash table for regions and/or obstacle directions?
//Likely also need to calculate angle that joysticks are telling drone to move at

struct
{
	uint16_t measurements[450];
	uint16_t index_adjustment;
} dists;

static CA_Pair sectorPairs[NUM_SECTORS];
static CA_Pair prevPairs[NUM_SECTORS];
static uint16_t hazardRating[NUM_SECTORS];

void initCA(PIDController* sectorControllers, float P, float I, float D)
{
	for(uint8_t i = 0; i < NUM_SECTORS; ++i)
	{
		prevPairs[i].angle = -1.0f;
		prevPairs[i].distance = 99999.0f;
		prevPairs[i].index = -1;
		sectorPairs[i].angle = -1.0f;	   //Can check for angle = -1 to see if sector has not been updated
		sectorPairs[i].distance = 99999.0f;//Just set to large value so that it is replaced quickly
		sectorPairs[i].index = -1;
		sectorControllers[i].PGain = P;
		sectorControllers[i].IGain = I;
		sectorControllers[i].DGain = D;

		sectorControllers[i].upperLimit = MAX_ADJ;
		sectorControllers[i].lowerLimit = -MAX_ADJ;

		sectorControllers[i].tau = 1.0;
		sectorControllers[i].error = 0.0;
		sectorControllers[i].PTerm = 0.0;
		sectorControllers[i].ITerm = 0.0;
		sectorControllers[i].DTerm = 0.0;
		sectorControllers[i].prev_measurement = 0.0;
	}

	memset(dists.measurements, 0xFFFF, 450*sizeof(uint16_t));
}

//Need to make yaw rotating CCW - and CW + -> For index adjustments
//measurements index needs to adjust based on yaw angle. If the drone rotates 80 degrees CCW (-80), then the index for 0deg (0) needs to
//map to the index for 280deg (350) --> index_adjustment = ((uint16_t)roundf(yawAngle / 0.8f) + 450 ) % 450
//May want to adjust distance measurements based on roll/pitch angle and assuming measurements are hypotinuse

//obstacle detection function should be replaced by, or use, DMA
void obstacleDetection(uint16_t * packet, float yawAngle)//Change ld19 getMeasurements to give full data back too
{
	//dists.index_adjustment = ((uint16_t)roundf(yawAngle / 0.8f) + 450 ) % 450;
	dists.index_adjustment = 0;//Until yaw can be accurately estimated, this is not worth using.
							   //Should work fine without it, measurements will need time to update after drone rotates though
							   //since the angles are relative to the drone, but the objects obviously are not
	float beginAngle = ((float)(packet[0]) / 100.0f);		//Convert from .01 deg to deg. Var for angle to begin displaying
	//float stopAngle = ((float)(packet[1]) / 100.0f);		//Convert from .01 deg to deg. Var for end angle of display
	uint16_t beginIndex = ((uint16_t)(roundf(beginAngle / 0.8f) + dists.index_adjustment) % 450);
	//uint16_t endIndex = ((uint16_t)(roundf(beginAngle / 0.8f) + dists.index_adjustment) % 450);
	//uint8_t numData = 12;
	uint16_t m_index = beginIndex;//Index for measurements
	uint8_t sectorIndex = 0;

	/*if(beginIndex + 12 > 449)
	{
		numData = 450 - beginIndex;
		for(uint8_t i = 0; i < numData; ++i)
		{
			dists.measurements[m_index] = packet[2 + i]//Store recent measurement at angle. Want indicies (5,4) (7,6) ... (27, 26)
			m_index++;
		}
		HAL_DMA_Start(&hdma_memtomem_dma1_stream5, &packet[numData],&dists.measurements[0], (12 - numData));
	}
	else
		HAL_DMA_Start(&hdma_memtomem_dma1_stream5, &packet[2],&dists.measurements[beginIndex], 12);*/



	for(uint8_t i = 2; i < 14; ++i)
	{
		dists.measurements[m_index] = packet[i];//Store recent measurement at angle.

		sectorIndex = (uint8_t)floor(((float)m_index * 0.8f) / ANGLE_TO_SECTOR_DIV);//Calc sector angle is part of

		//Print Objects Greater than 3 meters away
		//if(dists.measurements[m_index] >= 3000)
			//printf("Distance: %d, Angle: %f\r\n", dists.measurements[m_index], (float)m_index * 0.8f);

		if(sectorIndex >= NUM_SECTORS)//Can remove this later, or combine to just not update if invalid
		{
			printf("Error: sectorIndex Larger than Sector Pair array\r\n");
		}//Only update if obj is closer; distance at cur angle changed; and if new angle is greater than 200mm (beyond frame of drone)
		else if((sectorPairs[sectorIndex].distance > dists.measurements[m_index] || m_index == sectorPairs[sectorIndex].index) && dists.measurements[m_index] > 400)//Update closest distance for each sector
		{
			prevPairs[sectorIndex].angle = sectorPairs[sectorIndex].angle;
			prevPairs[sectorIndex].distance = sectorPairs[sectorIndex].distance;
			prevPairs[sectorIndex].index = sectorPairs[sectorIndex].index;
			sectorPairs[sectorIndex].angle = (float)m_index * 0.8f;
			sectorPairs[sectorIndex].distance = dists.measurements[m_index];
			sectorPairs[sectorIndex].index = m_index;
		}

		m_index = (m_index + 1) % 450;
	}
}


void hazardAssessment(float yawAngle, float speed, float direction)
{
	//Should only need to check the array of closest objs in the sectors
	//Remember to adjust index if accessing measurements
	for(uint8_t i = 0; i < NUM_SECTORS; ++i)
	{
		//Give hazard rating to sectors based on change in distance to the obj and distance to the obj
		//Does PID cover the change in distance?
		//Do not compare with initial sector vals (angle = -1 or index = -1)
	}

	//Keep an array of previous measurements and check which ones got closer (cur - prev).
	//The ones that got closer are more critical
}

void obstacleAvoidance(PIDController* sectorControllers, uint16_t* sectorAdjustments, float sampleTime)
{
	for(uint8_t i = 0; i < NUM_SECTORS; ++i)
	{
		//Since each sector has the angle of the closest obstacle, should base motor adjustments based on which motors the angle is closest to
		if(sectorPairs[i].distance <= MIN_DIST)
			sectorAdjustments[i] = PID_Update(&sectorControllers[i], sectorPairs[i].distance, MIN_DIST, sampleTime);
		else
			sectorAdjustments[i] = 0;
	}
}

