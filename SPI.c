/** \brief SPI implementation for RBE 3001
 * \file SPI.c
 *
 * Implements SPI functions for RBE 3001
 *
 * \date 7-Feb-2015
 * \author bnurbekov@wpi.edu wdhunt@wpi.edu
 */


#include <RBELib/RBELib.h>

/**
 * \brief Initializes the SPI bus for communication with all of the
 * 		  SPI devices.
 */
void initSPI() {
	// Set MOSI, SCK and SS'es output, all others input
	SPI_MOSI_DDR = OUTPUT;
	SPI_MISO_DDR = INPUT;
	SPI_SCK_DDR = OUTPUT;
	SPI_MASTER_SS = HIGH;
	DAC_SS_ddr = OUTPUT;

	AUX_DAC_SS_ddr = OUTPUT;
	ENCODER_SS_0_ddr = OUTPUT;
	ENCODER_SS_1_ddr = OUTPUT;
	ENCODER_SS_2_ddr = OUTPUT;
	ENCODER_SS_3_ddr = OUTPUT;
	SPARE_SS_ddr = OUTPUT;

	// Pull all slave selects low
	DAC_SS = HIGH;
	AUX_DAC_SS = HIGH;
	ENCODER_SS_0 = HIGH;
	ENCODER_SS_1 = HIGH;
	ENCODER_SS_2 = HIGH;
	ENCODER_SS_3 = HIGH;
	PORTCbits._P0 = HIGH; //spare ss

	// Enable SPI, Master, set clock rate fck/16
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);
}

/**
 * \brief Send and receive a byte out of the MOSI line.
 *		  Please note that even if you do not want to receive any data back
 *		  from a SPI device, the SPI standard requires you still receive something
 *		  back even if it is junk data.
 *
 * \param data The byte to send down the SPI bus
 * \return value The byte shifted in during transception
 */
unsigned char spiTransceive(BYTE data) {
	// Start transception
	SPDR = data;

	// Wait for transception complete
	while(!(SPSR & (1<<SPIF)));

	return SPDR;
}
