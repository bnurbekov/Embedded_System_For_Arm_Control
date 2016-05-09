/** \brief Kinematics implementation for RBE 3001
 * \file kinematics.c
 *
 * Implements kinematics functions for RBE 3001
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */


#include <math.h>
#include "kinematics.h"

/**
 * \brief Converts degrees to radians.
 *
 * \param degrees An angle in degrees
 * \return The angle in radians
 */
float convertToRadians(int degrees) {
	return (degrees / 180.0) * M_PI;
}

/**
 * \brief Converts radians to degrees.
 *
 * \param radians An angle in radians
 * \return The angle in degrees
 */
int convertToDegrees(float radians) {
	return (radians / M_PI) * 180;
}

/**
 * \brief Finds the X and Y coordinates of the tip based on the arm angles. Applies forward kinematics.
 *
 * \param coordinates The array that will hold the arm coordinates (two elements: x, y)
 * \param theta1 The angle of the lower link
 * \param theta2 The angle of the upper link
 * \param link1 The length of the lower link
 * \param link2 The length of the upper link
 */
void getCoordinatesForAngles(float* coordinates, int theta1, int theta2,
		float link1, float link2) {
	float theta1Rad = convertToRadians(theta1);
	float theta2Rad = convertToRadians(theta2);

	coordinates[0] = link1 * cos(theta1Rad)
			+ link2 * cos(theta1Rad + theta2Rad);
	coordinates[1] = link1 * sin(theta1Rad)
			+ link2 * sin(theta1Rad + theta2Rad);
}

/**
 * \brief Finds the angles for the set of X and Y coordinates. Applies inverse kinematics techniques.
 *
 * \param angles The array that will hold the arm angles in degrees (two elements; theta1, theta2)
 * \param X The x-coordinate to target
 * \param Y the y-coordinate to target
 * \param link1 The length of the lower link
 * \param link2 The length of the upper link
 */
void getAnglesForCoordinates(int* angles, float X, float Y, float link1,
		float link2) {
	float Bsquare = pow(X, 2) + pow(Y, 2);
	float q1 = atan2f(Y, X);
	float q2 = acos((pow(link1, 2) - pow(link2, 2) + Bsquare)/(2*link1*sqrt(Bsquare)));

	angles[0] = convertToDegrees(q1 + q2);
	angles[1] = convertToDegrees(acos((pow(link1, 2) + pow(link2, 2) - Bsquare)/(2*link1*link2))) - 180;
}
