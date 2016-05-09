/*
 * main.h
 *
 *  Created on: Feb 19, 2015
 *      Author: Batyr
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "RBELib/RBELib.h"
#include "timestamp.h"
#include "motorOutput.h"
#include "control.h"

#define DISTANCE_FROM_X_AXIS_TO_BLOCK_TIP 8
#define DISTANCE_FROM_ARM_TO_BACKWALL 190
#define DISTANCE_FROM_IR_SENSORS_TO_FURTHER_END_OF_CONVEYOR 206
#define DISTANCE_BETWEEN_IR_SENSORS 100
#define DISTANCE_FROM_HANDLE_TO_BLOCK_EDGE 13
#define FURTHEST_DETECTABLE_POINT 190
#define DISTANCE_BETWEEN_ARM_AND_IR_SENSOR 30
#define AVERAGE_ARM_GRABBING_DELAY 90

//detect the object only after 5 consistent readings from IR sensor (to filter out spikes)
#define ENTRY_NOISE_COUNT 3
#define EXIT_NOISE_COUNT 3

#define LOWER_BOUND_OF_HEAVY_BLOCK_LIFTING_TIME 3370
#define IR_SAMPLES_ARRAY_SIZE 5

#define EMA_ALPHA 0.75

#define GRIPPER_CHANNEL 0
#define CONVEYOR_CHANNEL 1
#define LOWER_LINK_CURR_SENSE_CHANNEL 0
#define UPPER_LINK_CURR_SENSE_CHANNEL 1
#define LOWER_LINK_POT_CHANNEL 2
#define UPPER_LINK_POT_CHANNEL 3
#define IR_SENSOR1_CHANNEL 6
#define IR_SENSOR2_CHANNEL 7

#define COMMAND_BUFFER_SIZE 30

// Configuration constants
const BOOL beltActive = 1;
const BOOL dataLoggingActive = 0;
const BOOL debugModeEnabled = 1;
const BOOL speedEstimationEnabled = 0;

//block speed in mm/ms
float blockSpeed;
//distance of the block handle in mm from the left side of the conveyer belt
unsigned int blockDistance;
//the weight of the block
enum {Light, Heavy} blockWeight;
//the time that will pass before grabbing
int countDown;
float positionalTolerance;

volatile int gripperInput = 0;
const int conveyorInput = 0;

volatile int irSensorReadings[2] = { 0 };
volatile int irSensorEMA[2] = { 0 };
volatile int currentReading;
volatile int upperJointADCCounts;
volatile int lowerJointADCCounts;
volatile BOOL infoUpdated = FALSE;

volatile BOOL commandWasReceived = FALSE;
volatile char command[COMMAND_BUFFER_SIZE];
volatile int commandIndex = 0;

volatile unsigned int pTermValue;

enum state {
	DetectDistanceAndSpeedOfBlock,
	GoToGrabbingPosition,
	Grab,
	LiftBlock,
	GoToInitialWeightingPosition,
	DetermineIfWithinWeightingPosition,
	StartLiftingToDetermineWeight,
	DetermineWeight,
	PlaceBlockIntoPredefinedLocation,
	DropBlock,
	GoToStartingPosition,
	Test,
	WaitForDelayToElapse,
	WaitForArmToReachPosition,
	MATLABControl,
	Idle,
	Calibration
};

enum state currentState = DetectDistanceAndSpeedOfBlock;
enum state nextStateAfterWait;

/**
 * \brief Find the angle of the given potentiometer.
 * \param  pot The pot to check.
 * \return Angle of the potentiometer in degrees
 */
int lowerPotAngle(int pot);

/**
 * \brief Initializes the timer1.
 */
void initTimer1();

/**
 * \brief Sends the ADC samples over serial
 */
void logData();

/**
 * \brief Initializes the system.
 */
void initSystem();

/**
 * \brief Writes the output to the external devices. Should be called every 10 ms in a while loop.
 */
void writeOutput();

/**
 * \brief Determines the distance and speed of the block from IR sensors. Switches current state
 * the block passes the second ir sensor.
 */
void determineDistanceAndSpeedOfBlock();

/**
 * \brief Commands arm to grab it.
 */
void grab();

/**
 * \brief Lifts the block once the gripper grabbed the handle.
 */
void liftAndDetermineWeight();

/**
 * \brief Determines the weight of the block while waiting for the arm to reach the position.
 */
void determineWeight();

/**
 * \brief Switches the current state to the Wait state. Waits in that state for the time specified
 * by delay and then switches to nextState.
 */
void waitForDelayToElapseAndAfterSwitchTo(unsigned int delay, enum state nextState);

/**
 * \brief Waits for the count down to elapse.
 */
void waitForDelayToElapse();

/**
 * \brief Waits for the arm to reach a predefined position and switches to the next state.
 */
void waitForArmToReachPositionAndAfterSwitchTo(float tolerance, enum state nextState);

/**
 * \brief Waits for the arm to reach the position that was set.
 */
void waitForArmToReachPosition();

/**
 * \brief Calculates exponential moving average.
 */
int calculateEMA(int currentValue, int prevValue, float alpha);

/**
 * \brief Returns the indication of whether the block passed the sensor.
 */
BOOL calculateDistance(int currentReading, int *outputDistance);

/**
 * Commands the arm to go to the angles specified by the command string.
 */
void sendCommandToArm();

#endif /* MAIN_H_ */
