/** \brief Pot Implementation for RBE 3001
 * \file pot.c
 *
 * Implements potentiometer functions for RBE 3001
 *
 * \date 21-Jan-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */

#include "RBELib/RBELib.h"

/**
 * \brief Find the angle of the given potentiometer.
 * \param pot The pot to check.
 * \return Angle of the potentiometer in degrees
 */
int potAngle(int pot) {
	int angle = (pot / 1024.0 * 252) - 156;
	return angle;
}

/**
 * \brief Find the voltage value of the given potentiometer.
 * \param  pot The pot to get the value of.
 * \return Voltage of potentiometer in millivolts
 */
int potVolts(int pot) {
	int volts = pot * 5000.0 / 1024;
	return volts;
}
