/** \brief USART Debugging Implementation for RBE 3001
 * \file USARTDebug.c
 *
 * Implements USART debugging functions for RBE 3001
 *
 * \author wdhunt@wpi.edu
 * \date 20-Jan-2015
 * \version 1
 */

#include "RBELib/RBELib.h"

/**
 *\brief Initializes USART1 as a print terminal to the PC.
 *
 * This function checks the incoming baudrate against the valid baudrates
 * from the ATMega644 data-sheet. If the baudrate is invalid, then the
 * DEFAULT_BAUD constant is used instead.
 *
 * \param baudrate The desired baudrate to set for USART1
 */
void debugUSARTInit(unsigned long baudrate) {

	// Choose default if improper baudrate is selected by caller
	if (!(baudrate == 2400 ||
		 baudrate == 4800 ||
		 baudrate == 9600 ||
		 baudrate == 14400 ||
		 baudrate == 19200 ||
		 baudrate == 28800 ||
		 baudrate == 38400 ||
		 baudrate == 57600 ||
		 baudrate == 76800 ||
		 baudrate == 115200))
		baudrate = DEFAULT_BAUD;

	unsigned int baudRegister = F_CPU/(baudrate*16UL)-1;

	// Set baud rate
	UBRR1H = (unsigned char) (baudRegister >> 8);
	UBRR1L = (unsigned char) baudRegister;

	// Enable RX and TX
	UCSR1C = (3<<UCSZ10);	// 8 data bits, 1 stop bit
	UCSR1B |= (1 << RXEN1) | (1 << TXEN1) | (1 << RXCIE1);
}

/**
 * \brief Sends one byte to the USART1 Tx pin (Transmits one byte).
 *
 * Note that this function is blocking. If low-latency serial is needed, consider implementing
 * interrupt services on the ATMega644's USART1.
 *
 * \param byteToSend The byte that is to be transmitted through USART1.
 *
 */
void putCharDebug(char byteToSend) {
	while (!(UCSR1A & (1 << UDRE1)));	// Wait until TX is ready to send

	UDR1 = byteToSend;
}

/**
 * \brief Receives one byte of data from the serial port (i.e. from the PC).
 *
 * Note that this function is blocking. If low-latency serial is needed, consider implementing
 * interrupt services on the ATMega644's USART1.
 *
 * \return byteReceived Character that was received on the USART.
 */
unsigned char getCharDebug(void) {
	while (!(UCSR1A & (1 << RXC1)));	// Wait for RX to complete

	return UDR1;
}
