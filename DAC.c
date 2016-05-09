/** \brief DAC implementation for RBE 3001
 * \file DAC.c
 *
 * Implements DAC functions for RBE 3001
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */


#include <RBELib/RBELib.h>

/**
 * \brief Set the DAC to the given value on the chosen channel.
 * \param  DACn The channel that you want to set.
 * \param SPIVal The value you want to set it to.
 */
void setDAC(int DACn, int SPIVal) {
	int i;
	BYTE dataArray[3];

	// Input validation
	SPIVal = (SPIVal < 0) ? 0 : SPIVal;
	SPIVal = (SPIVal > 4095) ? 4095 : SPIVal;

	// Populate the "send command and DAC address" byte (0b0011 (0x30) == 'Write and update DAC register n')
	dataArray[0] = 0x30 | DACn;
	// Get the first (starting from MSB) 8 bits of the data
	dataArray[1] = (BYTE) (SPIVal >> 4);
	// Get the rest 4 bits of the data (pad them to the left)
	dataArray[2] = (BYTE) (SPIVal << 4);

	// Now, when we have populated the array of data, we can actually send it.
	// Send the data (the clock is turned off automatically after each byte sent
	// and activated whenever the data is written to the data register)
	DAC_SS = LOW;
	for (i = 0; i < 3; i++) {
		spiTransceive(dataArray[i]);
	}
	DAC_SS = HIGH;
}
