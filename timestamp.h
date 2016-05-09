/** \brief Timestamping header for RBE 3001
 * \file timestamp.h
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */

#ifndef TIMESTAMP_H_
#define TIMESTAMP_H_

// Global timestamping
// Access with interrupts masked to prevent split signatures!
volatile unsigned int milliseconds;
volatile unsigned int seconds;

/**
 * \brief Updates a global timestamp variable.
 * \param millisecondsPassed The number of milliseconds that have passed since the last call
 */
void updateTimestamp(int millisecondsPassed);

/**
 * \brief Resets the timestamp. Can be used for the initialization of timestamp.
 */
void resetTimestamp();

/**
 * \brief Starts the stopwatch.
 */
void startStopWatch();

/**
 * \brief Stops the stopwatch.
 * \param outputArray A 2-element array that will contain the number of milliseconds and the number of seconds passed (in this order).
 */
void stopStopWatch(unsigned int *timePassed);

#endif /* TIMESTAMP_H_ */
