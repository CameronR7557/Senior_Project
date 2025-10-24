/*
 * collisionAvoidance.h
 *
 *  Created on: Feb 6, 2024
 *      Author: Cameron Robinson
 */

#ifndef FLIGHTCONTROLLER_COLLISIONAVOIDANCE_H_
#define FLIGHTCONTROLLER_COLLISIONAVOIDANCE_H_

#include "main.h"
#include "../FlightController/flightcontroller.h"

#define MAX_ADJ 100		//Max adjustment to motors from PID
#define NUM_SECTORS 8
#define ANGLE_TO_SECTOR_DIV (360.0f / NUM_SECTORS)
#define MIN_DIST 500.0f//610.0f //min allowable distance in mm. 610MM = 2ft

typedef struct
{
	float angle;
	float distance;
	int16_t index;
}CA_Pair;

void initCA(PIDController* sectorControllers, float P, float I, float D);
void obstacleDetection(uint16_t * packet, float yawAngle);
void hazardAssessment(float yawAngle, float speed, float direction);
void obstacleAvoidance(PIDController* sectorControllers, uint16_t* sectorAdjustments, float sampleTime);

#endif /* FLIGHTCONTROLLER_COLLISIONAVOIDANCE_H_ */
