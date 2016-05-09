/** \brief ADC Implementation for RBE 3001
 * \file ADC.c
 *
 * Implements ADC functions for RBE 3001
 *
 * \author wdhunt@wpi.edu
 * \date 21-Jan-2015
 * \version 1
 */

#include "RBELib/RBELib.h"

/**
 * \brief Initializes the ADC and make one channel active.
 * Uses polling to read the desired channel, so that the external
 * program can mask interrupts without inhibiting ADC operation.
 *
 * \param channel The ADC channel to initialize.
 */
void initADC(int channel) {
	changeADC(channel);
	ADCSRA = (1 << ADEN) | (7 << ADPS0);	// Enable ADC with a 128 clock divider
}

/**
 * \brief Run a conversion on and get the analog value from one ADC
 * channel if using polling.
 *
 * \param channel  The ADC channel to run a conversion on.
 * \return The 8-10 bit value returned by the ADC
 * conversion.  The precision depends on your settings and
 * how much accuracy you desire.
 */
unsigned short getADC(int channel) {
	changeADC(channel);
	ADCSRA |= (1 << ADSC);	// Enable ADC single conversion

	while (ADCSRA & (1 << ADSC));

	return ADC;
}

/**
 * \brief Change the channel the ADC is sampling if using interrupts.
 *
 * \param channel  The ADC channel to switch to.
 */
void changeADC(int channel) {
	ADMUX = ((channel & 0x07) << MUX0);
}
