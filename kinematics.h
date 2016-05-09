/** \brief Kinematics header for RBE 3001
 * \file kinematics.h
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */


#ifndef KINEMATICS_H_
#define KINEMATICS_H_


/**
 * \brief Converts degrees to radians.
 *
 * \param degrees An angle in degrees
 * \return The angle in radians
 */
float convertToRadians(int degrees);

/**
 * \brief Converts radians to degrees.
 *
 * \param radians An angle in radians
 * \return The angle in degrees
 */
int convertToDegrees(float radians);

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
		float link1, float link2);


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
		float link2);

#endif /* KINEMATICS_H_ */
