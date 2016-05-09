/** \brief Motor output implementation for RBE 3001
 * \file motorOutput.c
 *
 * Implements motor driving functions for RBE 3001
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */


#include "RBELib/RBELib.h"
#include "motorOutput.h"

/**
 * \brief Drives the motor with the power specified by the "value" input parameter.
 *
 * \param motorId The motor to drive (0 or 1)
 * \param value The value to send (-4095 to 4095)
 */
void driveMotor(int motorId, int value) {
	int dac1Id = (motorId == 0) ? 0 : 2;
	int dac2Id = (motorId == 0) ? 1 : 3;

	if (value >= 0) {
		setDAC(dac1Id, 0);
		setDAC(dac2Id, value);
	} else {
		setDAC(dac1Id, -value);
		setDAC(dac2Id, 0);
	}
}
