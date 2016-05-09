/*
 * Step2.c
 *
 *  Created on: Feb 13, 2015
 *      Author: Batyr
 */

#include "main.h"
#include "stdlib.h"

ISR(USART1_RX_vect) {
	char c = UDR1;

	if (commandIndex < COMMAND_BUFFER_SIZE) {
		if (c == '\r') {
			commandWasReceived = TRUE;
			c = '\0';
		}
	} else {
		commandIndex = 0;
	}

	command[commandIndex++] = c;

	if (commandWasReceived) {
		commandIndex = 0;
	}
}

/**
 * \brief Interrupt service for Timer1 Output Compare match
 *
 * Updates timestamp counters.
 */
ISR(TIMER1_COMPA_vect) {
	lowerJointADCCounts = getADC(LOWER_LINK_POT_CHANNEL);
	upperJointADCCounts = getADC(UPPER_LINK_POT_CHANNEL);
	pTermValue = getADC(4);

	currentAngles[0] = lowerPotAngle(lowerJointADCCounts);
	currentAngles[1] = potAngle(upperJointADCCounts);

	controlInputLower = calcPID('L', targetAngles[0], currentAngles[0]);
	controlInputUpper = calcPID('U', targetAngles[1], currentAngles[1]);

	irSensorReadings[0] = IRDist(IR_SENSOR1_CHANNEL);
	irSensorReadings[1] = IRDist(IR_SENSOR2_CHANNEL);

	irSensorEMA[0] = calculateEMA(irSensorReadings[0], irSensorEMA[0],
	EMA_ALPHA);
	irSensorEMA[1] = calculateEMA(irSensorReadings[1], irSensorEMA[1],
	EMA_ALPHA);

	currentReading = ((getADC(LOWER_LINK_CURR_SENSE_CHANNEL) / 1023.0 * 5000)
			- 2680);

	updateTimestamp(10);

	infoUpdated = TRUE;
}

/**
 * \brief Function created to measure the arm delay. It needs to be invoked only during the setup.
 */
void testArmDelay() {
	if (isWithinSetpoint(POSITIONAL_TOLERANCE)) {
		unsigned int timeElapsed[2];
		stopStopWatch(timeElapsed);
		printf("%d\n\r", timeElapsed[0] / 10 + timeElapsed[1] * 100);
	}
}

/**
 * \brief Find the angle of the given potentiometer.
 * \param  pot The pot to check.
 * \return Angle of the potentiometer in degrees
 */
int lowerPotAngle(int pot) {
	int angle = ((float) pot * 0.2308) - 58.3846;
	return angle;
}

/**
 * \brief Initializes the timer1.
 */
void initTimer1() {
	// Activate timer in CTC mode with /256 prescaler
	TCCR1B |= (1 << WGM12) | (1 << CS12);

	// Set timer1 counter initial value to 0
	TCNT1 = 0;

	// Enable CTC interrupt for Timer1
	TIMSK1 |= (1 << OCIE1A);

	// Initialize compare value
	OCR1A = 719; // meaning there is a tick every 10 ms (assuming the clock freq is 18.432MHz)
}

/**
 * \brief Sends the ADC samples over serial
 */
void logData() {
	int irSensorReadingsLocked[2];
	int currentReadingLocked;
	int upperJointADCCountsLocked;
	int lowerJointADCCountsLocked;

	cli();
	irSensorReadingsLocked[0] = irSensorReadings[0];
	irSensorReadingsLocked[1] = irSensorReadings[1];
	currentReadingLocked = currentReading;
	lowerJointADCCountsLocked = lowerJointADCCounts;
	upperJointADCCountsLocked = upperJointADCCounts;
	sei();

	printf("%d.%03d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n\r", seconds,
			milliseconds, lowerPotAngle(lowerJointADCCountsLocked),
			potAngle(upperJointADCCountsLocked), irSensorReadingsLocked[0],
			irSensorReadingsLocked[1], irSensorEMA[0], irSensorEMA[1],
			currentReadingLocked, currentState, nextStateAfterWait, countDown);

}

/**
 * \brief Initializes the system.
 */
void initSystem() {
	// initialize the timestamp
	resetTimestamp();

	debugUSARTInit(115200);
	initRBELib(); // Sets up standard output stream bindings with ATMega644 USART1
	// Initialize SPI and Timer1
	initSPI();
	initTimer1();
	// Setup ADCs
	initADC(LOWER_LINK_CURR_SENSE_CHANNEL); // Used for the current sense
	initADC(LOWER_LINK_POT_CHANNEL);
	initADC(UPPER_LINK_POT_CHANNEL); // Arm potentiometer
	initADC(4); // Used for P term calibration
	initADC(IR_SENSOR1_CHANNEL);	// Used for the first IR sensor
	initADC(IR_SENSOR2_CHANNEL); // Used for the second IR sensor

	//Initialize EMA's
	irSensorEMA[0] = IRDist(IR_SENSOR1_CHANNEL);
	irSensorEMA[1] = IRDist(IR_SENSOR2_CHANNEL);

	setConst('U', 335, 1914, 14.66); //P: 531/10.24, I: 20/1024, D: 7
	setConst('L', 250, 1667, 10.94);

	// Conveyer belt setup
	if (beltActive) {
		setServo(CONVEYOR_CHANNEL, conveyorInput);
	} else {
		setServo(CONVEYOR_CHANNEL, 90);
	}

	// Initial position of the arm
	//gotoAngles(90, 0);
	//gotoXY(180, -15); // <= initial position
	gotoXY(230, 20); // <= initial position
	//gotoXY(230, -10); // <= further end of the belt
	//gotoXY(185, -10); // <= closer end of the belt
	//gotoXY(208, -10); // <= center of the belt

	//enable interrupts
	sei();
}

/**
 * \brief Performs actions required in RBE 3001 lab 1
 *
 * Records the value in millivolts on the ADC at a constant sample rate of 225 Hz for one second,
 * after a button is pressed.
 *
 * \return Main function result (0 in success/default case)
 */
int main(void) {
	float measuredDistance;
	initSystem();

	while (TRUE) {
		if (infoUpdated) {
			switch (currentState) {
			case DetectDistanceAndSpeedOfBlock:
				determineDistanceAndSpeedOfBlock();
				break;
			case GoToGrabbingPosition:
				measuredDistance = DISTANCE_FROM_ARM_TO_BACKWALL
						+ (DISTANCE_FROM_IR_SENSORS_TO_FURTHER_END_OF_CONVEYOR
								- blockDistance);

				if (debugModeEnabled) {
					printf("Grabbing position: %d\n\r", (int) measuredDistance);
				}

//				if (measuredDistance < 220) {
//					gotoXY(200, -25);
//				}
//				else if (measuredDistance < 230) {
//					gotoXY(210, -25);
//				}
//				else if (measuredDistance < 240) {
//					gotoXY(219, -22);
//				}
//				else if (measuredDistance < 250) {
//					gotoXY(245, -13);
//				}
//				else if (measuredDistance < 260) {
//					gotoXY(256, -8);
//				}
//				else if (measuredDistance < 274) {
//					gotoXY(268, -11);
//				}
//				else {
				gotoXY(1.109 * measuredDistance - 29.01,
						0.255 * measuredDistance - 75.709);
//				}j

				currentState = Grab;
				break;
			case Grab:
				grab();
				break;
			case LiftBlock:
				gotoXY(230, 60); // <= initial position
				waitForArmToReachPositionAndAfterSwitchTo(10,
						GoToInitialWeightingPosition);
				break;
			case GoToInitialWeightingPosition:
				gotoAngles(0, 0);
				waitForArmToReachPositionAndAfterSwitchTo(10,
						StartLiftingToDetermineWeight);
				break;
			case StartLiftingToDetermineWeight:
				gotoAngles(90, 0);
				startStopWatch();
				currentState = DetermineWeight;
				break;
			case DetermineWeight:
				//override the lower control input to apply the constant voltage
				//while measuring the time to go from horizontal to vertical position
				controlInputLower = 1024;
				determineWeight();
				break;
			case PlaceBlockIntoPredefinedLocation:
				if (blockWeight == Heavy) {
					gotoXY(240, -5);
				} else {
					gotoAngles(0, 0);
				}
				waitForArmToReachPositionAndAfterSwitchTo(POSITIONAL_TOLERANCE,
						DropBlock);
				break;
			case DropBlock:
				gripperInput = 0;
				waitForDelayToElapseAndAfterSwitchTo(500, GoToStartingPosition);
				break;
			case GoToStartingPosition:
				gotoXY(230, 25); // <= initial position
				waitForArmToReachPositionAndAfterSwitchTo(POSITIONAL_TOLERANCE,
						DetectDistanceAndSpeedOfBlock);
				break;
			case WaitForDelayToElapse:
				waitForDelayToElapse();
				break;
			case WaitForArmToReachPosition:
				waitForArmToReachPosition();
				break;
			case MATLABControl:
				if (commandWasReceived) {
					sendCommandToArm();
					commandWasReceived = FALSE;
				}
				break;
			case Calibration:
				setConst('L', pTermValue, 0, 0);
				break;
			case Test:
				//Implement testing functionality here.
				break;
			case Idle:
			default:
				break;
			}

			writeOutput();

			infoUpdated = FALSE;
		}
	}
}

/**
 * \brief Determines the distance and speed of the block from IR sensors. Switches current state
 * the block passes the second ir sensor.
 */
void determineDistanceAndSpeedOfBlock() {
	unsigned int timeElapsed[2];

	static BOOL blockHasPassedIRSensor0 = FALSE;
	static int distanceIRSensor0;
	static int distanceIRSensor1;

	if (!blockHasPassedIRSensor0) {
		if (!calculateDistance(irSensorEMA[0], &distanceIRSensor0)) {
			return;
		} else {
			blockHasPassedIRSensor0 = TRUE;
			startStopWatch();
			if (debugModeEnabled) {
				printf("\n\rBlock has passed ir sensor 0!\n\r\n\r");
			}
		}
	}

	if (!calculateDistance(irSensorEMA[1], &distanceIRSensor1)) {
		return;
	}

	stopStopWatch(timeElapsed);

	//take the average of the block distances
	blockDistance = (distanceIRSensor1 + distanceIRSensor0
			+ 2 * DISTANCE_FROM_HANDLE_TO_BLOCK_EDGE) / 2; //we add 15 to determine distance to the handle

	if (speedEstimationEnabled) {
		blockSpeed = ((float) DISTANCE_BETWEEN_IR_SENSORS)
				/ (timeElapsed[0] + timeElapsed[1] * 1000);
	} else {
		blockSpeed = 0.021;
	}

	if (debugModeEnabled) {
		printf(
				"\n\rBlockDistance: %d, Block speed: %d (multiplied by 1000)\n\r\n\r",
				blockDistance, (int) (blockSpeed * 1000));
	}

	//switch current state
	waitForDelayToElapseAndAfterSwitchTo(
			DISTANCE_BETWEEN_ARM_AND_IR_SENSOR
					/ blockSpeed- AVERAGE_ARM_GRABBING_DELAY,
			GoToGrabbingPosition);

	blockHasPassedIRSensor0 = FALSE;
}

/**
 * \brief Returns the indication of whether the block passed the sensor.
 */
BOOL calculateDistance(int currentDistanceReading, int *outputDistance) {
	int i;
	BOOL hasBlockPassed = FALSE;
	static int entryNoiseCount = ENTRY_NOISE_COUNT;
	static int exitNoiseCount = EXIT_NOISE_COUNT;

	static int smallestDistanceSoFar = 9999;
	int averageDistance = 0;

	static int samples[IR_SAMPLES_ARRAY_SIZE];
	static int currentSampleArrayIndex = 0;
	static int numberOfSamplesStored = 0;

	if (currentDistanceReading < FURTHEST_DETECTABLE_POINT) {
		if (entryNoiseCount > 0) {
			entryNoiseCount--;
		} else {
			if (currentDistanceReading < smallestDistanceSoFar) {
				smallestDistanceSoFar = currentDistanceReading;
				//Put the distance reading into array of the smallest values
				samples[currentSampleArrayIndex] = smallestDistanceSoFar;

				currentSampleArrayIndex++;

				if (currentSampleArrayIndex >= IR_SAMPLES_ARRAY_SIZE) {
					currentSampleArrayIndex = 0;
				}

				if (numberOfSamplesStored < IR_SAMPLES_ARRAY_SIZE) {
					numberOfSamplesStored++;
				}

				//we are not going to exit until the readings consistently exceed smallest value by 15
				exitNoiseCount = EXIT_NOISE_COUNT;
			} else if (currentDistanceReading > smallestDistanceSoFar + 10) {
				if (exitNoiseCount > 0) { //exit only if the value is consistent
					exitNoiseCount--;
				} else {
					//calculate the average distance reading
					for (i = 0; i < numberOfSamplesStored; i++) {
						averageDistance += samples[i];
						if (debugModeEnabled) {
							printf("Array[%d]: %d\n\r", i, samples[i]);
						}
					}

					averageDistance = averageDistance / numberOfSamplesStored;

					if (debugModeEnabled) {
						printf(
								"Average distance: %d, Number of samples stored: %d\n\r",
								averageDistance, numberOfSamplesStored);
					}

					//reset static variables
					entryNoiseCount = ENTRY_NOISE_COUNT;
					exitNoiseCount = EXIT_NOISE_COUNT;
					smallestDistanceSoFar = 9999;
					currentSampleArrayIndex = 0;
					numberOfSamplesStored = 0;

					*outputDistance = averageDistance;
					hasBlockPassed = TRUE;
				}
			} else {
				//we are not going to exit until the readings consistently exceed smallest value by 15
				exitNoiseCount = EXIT_NOISE_COUNT;
			}
		}
	} else {
		entryNoiseCount = ENTRY_NOISE_COUNT;
	}

	return hasBlockPassed;
}

/**
 * \brief Writes the output to the external devices. Should be called every 10 ms in a while loop.
 */
void writeOutput() {
	driveMotor(0, -controlInputLower);
	driveMotor(1, controlInputUpper);
	setServo(GRIPPER_CHANNEL, gripperInput);

	if (dataLoggingActive) {
		logData();
	}
}

/**
 * \brief Commands arm to grab it.
 */
void grab() {
	if (isWithinSetpoint(POSITIONAL_TOLERANCE)) {
		gripperInput = 180;
		waitForDelayToElapseAndAfterSwitchTo(400, LiftBlock);
	}
}

/**
 * \brief Determines the weight of the block while waiting for the arm to reach the position.
 */
void determineWeight() {
	unsigned int timeElapsed[2];

	if (isWithinSetpoint(10)) {
		stopStopWatch(timeElapsed);

		if ((timeElapsed[0] + 1000 * timeElapsed[1])
				> LOWER_BOUND_OF_HEAVY_BLOCK_LIFTING_TIME) {
			blockWeight = Heavy;
		} else {
			blockWeight = Light;
		}

		if (debugModeEnabled) {
			printf("\n\rWeighing timer count: %d\n\r\n\r",
					timeElapsed[0] + 1000 * timeElapsed[1]);
		}

		currentState = PlaceBlockIntoPredefinedLocation;
	}
}

/**
 * \brief Switches the current state to the Wait state. Waits in that state for the time specified
 * by delay and then switches to nextState.
 */
void waitForDelayToElapseAndAfterSwitchTo(unsigned int delay,
		enum state nextState) {
	countDown = delay;
	currentState = WaitForDelayToElapse;
	nextStateAfterWait = nextState;
}

/**
 * \brief Waits for the arm to grab the block.
 */
void waitForDelayToElapse() {
	if (countDown <= 0) {
		currentState = nextStateAfterWait;
	} else {
		countDown -= 10;
	}
}

void waitForArmToReachPositionAndAfterSwitchTo(float tolerance,
		enum state nextState) {
	positionalTolerance = tolerance;
	currentState = WaitForArmToReachPosition;
	nextStateAfterWait = nextState;
}

/**
 * \brief Waits for the arm to reach the position that was set.
 */
void waitForArmToReachPosition() {
	if (isWithinSetpoint(positionalTolerance)) {
		currentState = nextStateAfterWait;
	}
}

/**
 * \brief Calculates exponential moving average.
 */
int calculateEMA(int currentValue, int prevValue, float alpha) {
	return alpha * currentValue + (1 - alpha) * prevValue;
}

/**
 * Commands the arm to go to the angles specified by the command string.
 */
void sendCommandToArm() {
	int angle0;
	int angle1;
	int i = 0;

	angle0 = atoi((const char *) command);
	while (command[i] != ' ' || i > commandIndex) {
		i++;
	}
	angle1 = atoi((const char *) (command + i));
	printf("%d, %d\n\r", angle0, angle1);

	gotoAngles(angle0, angle1);
}
