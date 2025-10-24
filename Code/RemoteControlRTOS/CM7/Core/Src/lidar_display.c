/*
 * lidar_display.c
 *
 *  Created on: Nov 25, 2023
 *      Author: robin
 */
#include <stdint.h>
#include "lidar_display.h"

void displayData(ili9341_t *lcd, uint16_t startAngle, uint16_t endAngle, uint8_t* data, uint8_t len)
{
	//printf("StartAngle: %d, EndAngle: %d\r\n", startAngle, endAngle);
	float beginAngle = ((float)(startAngle) / 100.0f) * DEG_TO_RAD;		//Convert from .01 deg to deg. Var for angle to begin displaying
	float stopAngle = ((float)(endAngle) / 100.0f) * DEG_TO_RAD;		//Convert from .01 deg to deg. Var for end angle of display
	float deltaAngle = stopAngle - beginAngle;							//Var for change in angle b/w measurements
	uint16_t x = 0;														//Var for x coord of LCD
	uint16_t y = 0;														//Var for y coord of LCD
	uint16_t x0 = 0;
	uint16_t y0 = 0;

	if(deltaAngle < 0)		//Only place where delta is - is when completing a rotation
		deltaAngle += 360.0f;

	deltaAngle = deltaAngle/11.0f;

	if(deltaAngle < 1)
	{
		for(uint8_t i = 0; i < 12; ++i)
		{
			//NOTE: May need to multiply trig funcs for y calculations by -1 if origin is top-left corner

			//Clear lines
			x0 = round(3 * sinf(beginAngle)) + 200;//switch sin and cos since 0 deg is vertical
			y0 = round(-3 * cosf(beginAngle) + (LCD_HEIGHT/2));
			x = round((LCD_HEIGHT/2) * sinf(beginAngle) + 200);
			y = round(-(LCD_HEIGHT/2) * cosf(beginAngle) + (LCD_HEIGHT/2));
			//ili9341_draw_line(lcd, ILI9341_BLUE, x0, y0, x, y);
			ili9341_draw_line(lcd, ILI9341_WHITE, x0, y0, x, y);

			if(data[i] <= 120)
			{
				//Get location of obj
				//x = LIDAR_LEFT + (uint16_t)(round(fabs((LCD_WIDTH/2) + ((float)(data[i]) * sinf(beginAngle)))));
				x = (uint16_t)(round(fabs((float)(data[i]) * sinf(beginAngle) + 200)));
				y = (uint16_t)(round(fabs((LCD_HEIGHT/2) - ((float)(data[i]) * cosf(beginAngle)))));
				//printf("X: %d, Y: %d, Begin: %0.3f, End: %0.3f, Delta: %0.3f\r\n", x, y, beginAngle, stopAngle, deltaAngle);
				//Draw obj at location
				ili9341_draw_pixel(lcd, ILI9341_BLACK, x, y);
			}
			beginAngle += deltaAngle;
		}
	}
}

/*void displayData(ili9341_t *lcd, uint16_t startAngle, uint16_t endAngle, uint8_t* data, uint8_t len)
{
	//printf("StartAngle: %d, EndAngle: %d\r\n", startAngle, endAngle);
	float beginAngle = ((float)(startAngle) / 100.0f) * DEG_TO_RAD;		//Convert from .01 deg to deg. Var for angle to begin displaying
	float stopAngle = ((float)(endAngle) / 100.0f) * DEG_TO_RAD;		//Convert from .01 deg to deg. Var for end angle of display
	float deltaAngle = stopAngle - beginAngle;							//Var for change in angle b/w measurements
	uint16_t x = 0;														//Var for x coord of LCD
	uint16_t y = 0;														//Var for y coord of LCD
	uint16_t x0 = 0;
	uint16_t y0 = 0;

	if(deltaAngle < 0)		//Only place where delta is - is when completing a rotation
		deltaAngle += 360.0f;

	deltaAngle = deltaAngle/11.0f;

	for(uint8_t i = 0; i < 12; ++i)
	{
		//NOTE: May need to multiply trig funcs for y calculations by -1 if origin is top-left corner

		//Clear lines
		x0 = LIDAR_LEFT + round(1 * sinf(beginAngle) + (LCD_WIDTH/2));//switch sin and cos since 0 deg is vertical
		y0 = round(-1 * cosf(beginAngle) + (LCD_HEIGHT/2));
		x = LIDAR_LEFT + round((LCD_HEIGHT/2) * sinf(beginAngle) + (LCD_WIDTH/2));
		y = round(-(LCD_HEIGHT/2) * cosf(beginAngle) + (LCD_HEIGHT/2));
		ili9341_draw_line(lcd, ILI9341_WHITE, x0, y0, x, y);

		if(data[i] <= 120)
		{
			//Get location of obj
			x = LIDAR_LEFT + (uint16_t)(round(fabs((LCD_WIDTH/2) + ((float)(data[i]) * sinf(beginAngle)) * ((float)(LCD_HEIGHT) / (float)(MAX_DISPLAY_DIST)))));
			y = (uint16_t)(round(fabs((LCD_HEIGHT/2) - ((float)(data[i]) * cosf(beginAngle)) * ((float)(LCD_HEIGHT) / (float)(MAX_DISPLAY_DIST)))));
			//printf("X: %d, Y: %d, Begin: %0.3f, End: %0.3f, Delta: %0.3f\r\n", x, y, beginAngle, stopAngle, deltaAngle);
			//Draw obj at location
			ili9341_draw_pixel(lcd, ILI9341_BLACK, x, y);
		}
		beginAngle += deltaAngle;
	}
}
 * */
