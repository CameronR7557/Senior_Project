
#ifndef FILTERS_COMPLEMENTARYFILTER_H_
#define FILTERS_COMPLEMENTARYFILTER_H_

#include "../FlightController/flightcontroller.h"

struct eulerAngles
{
	float roll;
	float pitch;
	float yaw;
};

void ComplementaryFilter(struct bmi08x_sensor_data_f* acc, struct bmi08x_sensor_data_f* gyro, struct eulerAngles* EulerAngles);


#endif /* FILTERS_COMPLEMENTARYFILTER_H_ */
