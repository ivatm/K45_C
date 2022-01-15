/*
 * uart.h
 *
 *  Created on: 21 ����. 2021 �.
 *      Author: Oliva
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// UART Definitions
// B0,    B50,   B75,   B110,   B134,   B150,   B200,    B300,    B600, B1200, B1800,
// B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800
#define kBoadRate_Default   B9600
#define kBuffInput_length   255
#define kBuff_In_Out_length 100
#define kCommand_length     10

// The general terminal interface that
// is provided to control asynchronous communications ports
struct termios serial;

// Allocate memory for In/Out string.
char buffer_out[kBuff_In_Out_length];
char received_data[kBuff_In_Out_length];

// Allocate memory for read buffer, set size according to your needs
char buffer_in[kBuffInput_length];
char* pBufferWritePointer;
char* pBufferReadPointer;

uint16_t ByteSReceived;

// file descriptor of terminal
int fd_PC_Communication;

// Type define
typedef struct
{
   uint8_t cComm;      // Command
   uint8_t cLength;    // Number of byte
   uint8_t cData[kCommand_length];  // Data not longer then 10 byte
} sComm_full_structure;

typedef enum
{
   eLookForStartTelegramm = 0,
   eReadingCommand,
   eReadingTelegramm,
} CommunicationState_enum;


#endif /* INC_UART_H_ */
