/*
 * control.c
 *
 *  Created on: Feb 14, 2015
 *      Author: Batyr
 */

#include "math.h"
#include "control.h"
#include "kinematics.h"

void updateTargetAngles(int *angles) {
	targetAngles[0] = angles[0];
	targetAngles[1] = angles[1];

	getCoordinatesForAngles(targetCoordinates, angles[0], angles[1], LOWER_LINK_LENGTH, UPPER_LINK_LENGTH);
}

/**
 * \brief Updates the arm angle setpoints to reflect the current target endpoint position.
 */
void updateTargetCoordinates(float *coordinates) {
	targetCoordinates[0] = coordinates[0];
	targetCoordinates[1] = coordinates[1];

	// Compute inverse kinematics
	getAnglesForCoordinates(targetAngles, coordinates[0], coordinates[1], LOWER_LINK_LENGTH, UPPER_LINK_LENGTH);
}

/**
 * \brief Determines whether the endpoint is currently within an acceptable radius of the setpoint
 * \param tolerance The acceptable deviation from the setpoint
 * \return True if the endpoint is within the tolerance of the setpoint; false otherwise
 */
char isWithinSetpoint(float tolerance) {
	getCoordinatesForAngles(currentCoordinates, currentAngles[0], currentAngles[1], LOWER_LINK_LENGTH, UPPER_LINK_LENGTH);

	return sqrt(pow(targetCoordinates[0] - currentCoordinates[0], 2) + pow(targetCoordinates[1] - currentCoordinates[1], 2)) <= tolerance;
}
