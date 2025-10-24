/*
 * lidar_display.h
 *
 *  Created on: Nov 25, 2023
 *      Author: robin
 */

#ifndef INC_LIDAR_DISPLAY_H_
#define INC_LIDAR_DISPLAY_H_

#include <stdint.h>
#include "ili9341_gfx.h"

#define BLACK 0xff
#define WHITE 0x00
#define PI 3.14159f
#define DEG_TO_RAD	(float)(PI/180.0F)
#define LCD_WIDTH 		320
#define LCD_HEIGHT 		240

#define LIDAR_LEFT		79
#define LIDAR_RIGHT		319
#define LIDAR_TOP		0
#define LIDAR_BOTTOM	239

void displayData(ili9341_t *lcd, uint16_t startAngle, uint16_t endAngle, uint8_t* data, uint8_t len);

#endif /* INC_LIDAR_DISPLAY_H_ */
