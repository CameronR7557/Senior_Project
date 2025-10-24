#ifndef LD19_H_
#define LD19X_H_

#include "stdint.h"
#include "../FlightController/flightcontroller.h"
#define POINT_PER_PACK 12

#define HEADER 0x54

#define METER_DISPLAY 	2.0f
#define LCD_WIDTH 		320
#define LCD_HEIGHT 		240

#define LIDAR_LEFT		79
#define LIDAR_RIGHT		319
#define LIDAR_TOP		0
#define LIDAR_BOTTOM	239

typedef struct __attribute__((packed))
{
	uint16_t distance;
	uint8_t  intensity;
} LidarPointStructDef;

typedef struct   __attribute__((packed))
{
	uint8_t 			header;
	uint8_t 			ver_len;
	uint16_t 			speed;
	uint16_t 			start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t    		end_angle;
	uint16_t     		timestamp;
	uint8_t     		crc8;
}LiDARFrameTypeDef;

uint8_t CalCRC8(uint8_t *packet, uint8_t len);
uint8_t CalCRC8_2(const uint8_t *data, uint16_t data_len);
void ParsePacket(uint8_t* packet, LiDARFrameTypeDef* frame);
void getLiDARMeasurements(uint8_t* packet, uint8_t* dataPacket, uint16_t* fullDataPacket);

#endif









