/* 
 * File:   MMA8452.h
 * Author: Jim
 *
 * Created on January 16, 2015, 11:42 AM
 */

#ifndef MMA8452_H
#define	MMA8452_H

#include "GenericTypeDefs.h"

//Define a few of the registers that we will be accessing on the MMA8452
#define OUT_X_MSB 0x01
#define XYZ_DATA_CFG  0x0E
#define WHO_AM_I   0x0D
#define CTRL_REG1  0x2A
#define ACCELEROMETER_ID 0x1D
#define MAX_I2C_REGISTERS 6

#define GSCALE 2 // Sets full-scale range to +/-2, 4, or 8g. Used to calc real g values.
#define I2C_CLOCK_FREQ             5000
//#define EEPROM_I2C_BUS              I2C3
//#define EEPROM_ADDRESS              0x1D

#define I2C_READ 1
#define I2C_WRITE 0

extern BOOL TransmitOneByte(unsigned char data);
extern BOOL StartTransfer(BOOL restart);
extern void StopTransfer(void);
extern unsigned char initMMA8452(void);
extern unsigned char setRegister(unsigned char deviceID, unsigned char deviceREGISTER);
extern unsigned char sendREADcommand(unsigned char deviceID);
extern unsigned char readRegisters(unsigned char deviceID, unsigned char deviceREGISTER, unsigned char numRegisters, unsigned char *registerPtr);
extern unsigned char writeByteToRegister(unsigned char deviceID, unsigned char deviceREGISTER, unsigned char dataByte);
short convertValue(unsigned char MSBbyte, unsigned char LSBbyte);
short getTwosComplement(unsigned char MSBbyte, unsigned char LSBbyte);
unsigned char initMMA8452(void);


#endif	/* MMA8452_H */

