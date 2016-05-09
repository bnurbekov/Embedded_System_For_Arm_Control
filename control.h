/*
 * controlVariables.h
 *
 *  Created on: Feb 14, 2015
 *      Author: Batyr
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#define POSITIONAL_TOLERANCE 6

#define LOWER_LINK_LENGTH 150
#define UPPER_LINK_LENGTH 150

volatile int targetAngles[2];
volatile int currentAngles[2];

float targetCoordinates[2];
float currentCoordinates[2];

volatile int controlInputLower;
volatile int controlInputUpper;

/**
 * \brief Properly updates target angles.
 */
void updateTargetAngles(int *angles);

/**
 * \brief Properly updates target coordinates.
 */
void updateTargetCoordinates(float *coordinates);

/**
 * \brief Determines whether the endpoint is currently within an acceptable radius of the setpoint
 * \param tolerance The acceptable deviation from the setpoint
 * \return True if the endpoint is within the tolerance of the setpoint; false otherwise
 */
char isWithinSetpoint(float tolerance);

#endif /* CONTROL_H_ */
