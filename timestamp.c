/** \brief Timestamping implementation for RBE 3001
 * \file timestamp.c
 *
 * Implements timestamping functions for RBE 3001
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */

#include "timestamp.h"

// The arguments
unsigned int stopWatchMilliseconds = 0;
unsigned int stopWatchSeconds = 0;

/**
 * \brief Updates a global timestamp variable.
 * \param millisecondsPassed The number of milliseconds that have passed since the last call
 */
void updateTimestamp(int millisecondsPassed) {
	milliseconds += millisecondsPassed;

	if (milliseconds > 999) {
		seconds++;
		milliseconds = milliseconds - 1000;
	}
}

/**
 * \brief Resets the timestamp. Can be used for the initialization of timestamp.
 */
void resetTimestamp() {
	milliseconds = 0;
	seconds = 0;
}

/**
 * \brief Starts the stopwatch.
 */
void startStopWatch() {
	stopWatchMilliseconds = milliseconds;
	stopWatchSeconds = seconds;
}

/**
 * \brief Stops the stopwatch. If the startStopWatch() was not executed prior to calling this function, it will return the time elapsed since the last reset.
 * \param timePassed A 2-element array that will contain the number of milliseconds and the number of seconds passed (in this order).
 */
void stopStopWatch(unsigned int *timePassed) {
	if (milliseconds >= stopWatchMilliseconds) {
		timePassed[0] = milliseconds - stopWatchMilliseconds;
		timePassed[1] = seconds - stopWatchSeconds;
	}
	else {
		timePassed[0] = 1000 - stopWatchMilliseconds + milliseconds;
		timePassed[1] = seconds - stopWatchSeconds - 1;
	}
}
