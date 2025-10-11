#ifndef _PAQUITO_
#define _PAQUITO_

#include <cmath>

struct WheelPos
{
	const char* const name;
	double x, y, roll;
} typedef WheelPos;

const double WHEEL_X = 0.0755 * 2;  // Distancia en x entre las llantas
const double WHEEL_Y = 0.1 * 2;     // Distancia en y entre las llantas
const double WHEEL_R = 0.0375;      // Radio de la llanta
const double WHEEL_ROLL = M_PI / 2;

WheelPos fl = WheelPos { "front_left_wheel", WHEEL_X/2, WHEEL_Y/2, WHEEL_ROLL };
WheelPos fr = WheelPos { "front_right_wheel", WHEEL_X/2, -WHEEL_Y/2, WHEEL_ROLL };
WheelPos rr = WheelPos { "rear_right_wheel", -WHEEL_X/2, -WHEEL_Y/2, WHEEL_ROLL };
WheelPos rl = WheelPos { "rear_left_wheel", -WHEEL_X/2, WHEEL_Y/2, WHEEL_ROLL };

#endif