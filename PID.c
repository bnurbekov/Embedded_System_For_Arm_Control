/** \brief PID implementation for RBE 3001
 * \file PID.c
 *
 * Implements PID functions for RBE 3001
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */

#include "RBELib/RBELib.h"
#include <math.h>
#include "kinematics.h"

#define ABS_MAX_OUTPUT 4095

const float Kg = 5;

pidConst pidConsts;
int deltaT = 10;
long integralAccums[2] = { 0 };
long prevErrors[2] = { 0 };

/**
 * \brief Sets the Kp, Ki, and Kd values for one link.
 *
 * \param link The link you want to set the values for (H or L).
 * \param Kp Proportional value.
 * \param Ki Integral value.
 * \param Kd Derivative value.
 */
void setConst(char link, float Kp, float Ki, float Kd) {
	if (link == 'U') {
		pidConsts.Kp_H = Kp;
		pidConsts.Ki_H = Ki;
		pidConsts.Kd_H = Kd;
	} else {
		pidConsts.Kp_L = Kp;
		pidConsts.Ki_L = Ki;
		pidConsts.Kd_L = Kd;
	}
}

/**
 * \brief Calculate the PID value.
 *
 * \param  link Which link to calculate the error for (Use 'U' and 'L').
 * \param setPoint The desired position of the link.
 * \param actPos The current position of the link.
 */
signed int calcPID(char link, int setPoint, int actPos) {
	long controlInput;
	long *integralAccumPtr;
	int *prevErrorPtr;

	float Kp;
	float Kd;
	float Ki;

	int error = setPoint - actPos;

	// Compute link integral and select location to log previous error
	if (link == 'U') {
		Kp = pidConsts.Kp_H;
		Ki = pidConsts.Ki_H;
		Kd = pidConsts.Kd_H;
		prevErrorPtr = &(prevErrors[0]);
		integralAccumPtr = &(integralAccums[0]);
	} else {
		Kp = pidConsts.Kp_L;
		Ki = pidConsts.Ki_L;
		Kd = pidConsts.Kd_L;
		prevErrorPtr = &(prevErrors[1]);
		integralAccumPtr = &(integralAccums[1]);
	}

	// Compute integral term saturation
	*integralAccumPtr += Ki * deltaT * (*integralAccumPtr);

	if (*integralAccumPtr > ABS_MAX_OUTPUT) {
		*integralAccumPtr = ABS_MAX_OUTPUT;
	} else if (*integralAccumPtr < -ABS_MAX_OUTPUT) {
		*integralAccumPtr = -ABS_MAX_OUTPUT;
	}

	// Build output value from PID component control inputs
	controlInput = (Kp * error + Ki * deltaT * (*integralAccumPtr)
			+ Kd / deltaT * (error - *prevErrorPtr));

	*prevErrorPtr = error;

	//TODO: implement gravity compensation

	// Make sure that the control input doesn't exceed the maximum DAC values
	if (controlInput > ABS_MAX_OUTPUT) {
		controlInput = ABS_MAX_OUTPUT;
	}
	if (controlInput < -ABS_MAX_OUTPUT) {
		controlInput = -ABS_MAX_OUTPUT;
	}

	return controlInput;
}
