/** \brief Motor output header for RBE 3001
 * \file motorOutput.h
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */

#ifndef MOTOROUTPUT_H_
#define MOTOROUTPUT_H_

/**
 * \brief Drives the motor with the power specified by the "value" input parameter.
 *
 * \param motorId The motor to drive (0 or 1)
 * \param value The value to send (-4095 to 4095)
 */
void driveMotor(int motorId, int value);

#endif /* MOTOROUTPUT_H_ */
