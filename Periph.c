/*
 * Periph.c
 *
 *  Created on: Feb 11, 2015
 *      Author: Batyr
 */

#include "RBELib/RBELib.h"

#define SPARE_SS PORTCbits._P0
#define LINEARIZATION_CONSTANT 1
#define SLOPE_INVERSE_CONSTANT 2914
#define VERTICALSHIFT_INVERSE_CONSTANT 5

/**
 * @brief Find the acceleration in the given axis (X, Y, Z).
 * @param  axis The axis that you want to get the measurement of.
 * @return gVal Value of  acceleration.
 *
 * @todo Create a function that is able to find the acceleration of a given axis.
 */
signed int getAccel(int axis) {
	signed int result = 0;
	char channel = axis;

	SPARE_SS = LOW;
	spiTransceive(0b00000110);
	result = spiTransceive(channel << 6);
	result = (result & 0x0F) << 8;
	result |= spiTransceive(0x00);
	SPARE_SS = HIGH;

	return result;
}

/**
 * @brief Read an IR sensor and calculate the distance of the block.
 * @param  chan The port that the IR sensor is on.
 * @return value The distance the block is from the sensor.
 *
 * @todo Make a function that is able to get the ADC value of the IR sensor.
 */
int IRDist(int chan) {
	/*
	return (((float) SLOPE_INVERSE_CONSTANT
			/ (getADC(chan) + VERTICALSHIFT_INVERSE_CONSTANT))
			- LINEARIZATION_CONSTANT) * 25.4;*/

	float counts = getADC(chan);

	return 0.0012f*(counts)*(counts) - 1.3675f*counts + 485.35f;

	//return getADC(chan);
}

/**
 * @brief Initialize the encoders with the desired settings.
 * @param chan Channel to initialize.
 *
 * @todo Make a function that can setup both encoder chips on the board.
 */
void initEnc(int chan) {
	if (chan != 0 && chan != 1)
		return;

	if (chan == 0)
		ENCODER_SS_0 = LOW;
	else if (chan == 1)
		ENCODER_SS_1 = LOW;

	// Send configuration bytes
	spiTransceive(0b10001000);
	spiTransceive(0b00000011);

	if (chan == 0)
		ENCODER_SS_0 = HIGH;
	else if (chan == 1)
		ENCODER_SS_1 = HIGH;
}

/**
 * @brief Reset the current count of the encoder ticks.
 * @param chan The channel to clear.
 *
 */
void resetEncCount(int chan) {
	if (chan != 0 && chan != 1)
		return;

	if (chan == 0)
		ENCODER_SS_0 = LOW;
	else
		ENCODER_SS_1 = LOW;

	spiTransceive(0b00100000);	// Clear the counter

	if (chan == 0)
		ENCODER_SS_0 = HIGH;
	else if (chan == 1)
		ENCODER_SS_1 = HIGH;

}

/**
 * @brief Finds the current count of one of the encoders.
 * @param chan Channel that the encoder is on that you would like to read.
 * @return count The current count of the encoder.
 */
signed long encCount(int chan) {
	signed long count = 0;

	if (chan != 0 && chan != 1)
		return -1;

	if (chan == 0)
		ENCODER_SS_0 = LOW;
	else
		ENCODER_SS_1 = LOW;
	// Send read-start byte
	spiTransceive(0b01100000);

	for (int i = 3; i >= 0; i--) {
		count |= spiTransceive(0b00000000) << i * 8;
	}

	if (chan == 0)
		ENCODER_SS_0 = HIGH;
	else if (chan == 1)
		ENCODER_SS_1 = HIGH;

	return count;
}
