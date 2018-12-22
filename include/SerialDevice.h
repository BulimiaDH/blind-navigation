/** This file is from Kyle Vogt's Roboteq Library:
 http://web.mit.edu/kvogt/www/roboteq.html
 */

#ifndef _SERIAL_DEVICE_H_
#define _SERIAL_DEVICE_H_

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <string>
#include <ext/stdio_filebuf.h>

#define PARITY_8N1 1
#define PARITY_7E1 2

#define BUFFERSIZE	255

/**
 * This class handles low level serial port I/O
 */

class Serial {
	public:

		/**
		 * Constructor.
		 * Initializes class variables.
		 */
		Serial();

		/** 
		 * Deconstructor.
		 * Called when class instance is destroyed.
		 */
		~Serial();

		/**
		 * Serial port initialization.
		 * Opens a port and returns the file descriptor
 		 * @return int - file descriptor (value of -1 means init failed)
		 */
		int initPort();

		/**
		 * Read the buffer.
		 * Polls the serial port for data and returns it to the host program
		 * @param data - string to hold the new data
		 * @param len number of bytes to read
		 * @return int - number of bytes collected
		 */
		int getData(char* data, int len);

		/** 
		 * Writes to the buffer.
		 * @param data - string containing data to send
		 * @return int - number of bytes sent
		 */
		int sendData(const unsigned char* data, int len);

		/**
		 * Reads a byte from the buffer.
		 * @return char - ASCII value of byte read, -1 if no byte read.
		 */
		char getChar();
		
		/**
		 Attempts to read an entire line from the serial port
		 \param data buffer to read into
		 \return number of bytes read (or -1 on error)
		 */
		int getLine(char* data);

		/**
		 * Sends a byte to the buffer.
		 * @param data - The char to send to serial port.
		 * @param return - Number of bytes sent.
		 */
		int sendChar(char data);

		/**
		 * Flush port.
		 * Clears all data that is currently pending in either the transmit or receive buffers.
		 */
		void flushPort();

		/** 
		 * Close port.
		 * Serial port communication closed and port freed.
		 */
		void closePort();

		int fd; /**< File descriptor for open port */
		std::string dev; /**< Device string for serial port */
		int baud; /**< Baud rate for serial port */
		int dataBits; /**< Number of data bits for serial port */
		int bufferSize; /**< Sets the size of the input buffer */
		int parity; /**< Style of communication (8N1 or 7E1) */
		char temp[256]; /**< Temporary storage for getChar() function */
		char buffer[256]; /**< Buffer to hold extra bytes received */
		int bufferIndex; /**< Location for next byte in buffer */
		int blocking; /**< Flag for opening port in blocking mode */
		char waitTime;
		
		struct termios orig_options;

	private:
};

#endif //_SERIAL_H
