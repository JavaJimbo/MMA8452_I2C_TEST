// #include "xc.h"
#include "Defs.h"
#include "MMA8452.h"
#include "GenericTypeDefs.h"
#include "I2C_EEPROM_PIC32.h"
#include <plib.h>

/*******************************************************************************
  Function:
    BOOL StartTransfer( BOOL restart )

  Summary:
    Starts (or restarts) a transfer to/from the EEPROM.

  Description:
    This routine starts (or restarts) a transfer to/from the EEPROM, waiting (in
    a blocking loop) until the start (or re-start) condition has completed.

  Precondition:
    The I2C module must have been initialized.

  Parameters:
    restart - If FALSE, send a "Start" condition
            - If TRUE, send a "Restart" condition

  Returns:
    TRUE    - If successful
    FALSE   - If a collision occured during Start signaling

  Example:
    <code>
    StartTransfer(FALSE);
    </code>

  Remarks:
    This is a blocking routine that waits for the bus to be idle and the Start
    (or Restart) signal to complete.
 *****************************************************************************/

BOOL StartTransfer(BOOL restart) {
    I2C_STATUS status;  
    
    // Send the Start (or Restart) signal
    if (restart) {
        I2CRepeatStart(EEPROM_I2C_BUS);
    } else {
        // Wait for the bus to be idle, then start the transfer
        while (!I2CBusIsIdle(EEPROM_I2C_BUS));

        if (I2CStart(EEPROM_I2C_BUS) != I2C_SUCCESS) {
            //DBPRINTF("Error: Bus collision during transfer Start\n");
            printf("Error: Bus collision during transfer Start\n");
            return FALSE;
        }
    }

    // Wait for the signal to complete
    do {
        status = I2CGetStatus(EEPROM_I2C_BUS);

    } while (!(status & I2C_START));
    
    return TRUE;
}

/*******************************************************************************
  Function:
    BOOL TransmitOneByte( UINT8 data )

  Summary:
    This transmits one byte to the EEPROM.

  Description:
    This transmits one byte to the EEPROM, and reports errors for any bus
    collisions.

  Precondition:
    The transfer must have been previously started.

  Parameters:
    data    - Data byte to transmit

  Returns:
    TRUE    - Data was sent successfully
    FALSE   - A bus collision occured

  Example:
    <code>
    TransmitOneByte(0xAA);
    </code>

  Remarks:
    This is a blocking routine that waits for the transmission to complete.
 *****************************************************************************/

BOOL TransmitOneByte(unsigned char data){
    // Wait for the transmitter to be ready
    while (!I2CTransmitterIsReady(EEPROM_I2C_BUS));

    // Transmit the byte
    if (I2CSendByte(EEPROM_I2C_BUS, data) == I2C_MASTER_BUS_COLLISION) {
        printf("Error: I2C Master Bus Collision\n");
        return FALSE;
    }

    // Wait for the transmission to finish
    while (!I2CTransmissionHasCompleted(EEPROM_I2C_BUS));

    return TRUE;
}


// This routine initiates I2C reads and writes
// It sends a START, then the DEVICE address,
// then the first REGISTER to read or write.

unsigned char sendREADcommand(unsigned char deviceID) {
    unsigned char startCounter = 100;

    while (startCounter--) {
        if (StartTransfer(TRUE))break;
        DelayMs(10);
    }

    if (!startCounter)return (FALSE);


    // Write device ID
    if (!TransmitOneByte((deviceID << 1) | I2C_READ))return (FALSE);

    // Verify that the byte was acknowledged
    if (!I2CByteWasAcknowledged(EEPROM_I2C_BUS))return (FALSE);

    return (TRUE);

}

unsigned char setRegister(unsigned char deviceID, unsigned char deviceREGISTER) {
    unsigned char startCounter = 100;

    while (startCounter--) {
        if (StartTransfer(FALSE))break;
        DelayMs(10);
    }

    if (!startCounter) {
        DelayMs(200);
        I2CStop(EEPROM_I2C_BUS);
        return (FALSE);
    }

    // Write device ID
    if (!TransmitOneByte((deviceID << 1) | I2C_WRITE))return (FALSE);

    // Verify that the byte was acknowledged
    if (!I2CByteWasAcknowledged(EEPROM_I2C_BUS))return (FALSE);

    // Write first register to be addressed
    if (!TransmitOneByte(deviceREGISTER))return (FALSE);


    // Verify that the byte was acknowledged
    if (!I2CByteWasAcknowledged(EEPROM_I2C_BUS))return (FALSE);

    return (TRUE);

}

unsigned char writeByteToRegister(unsigned char deviceID, unsigned char deviceREGISTER, unsigned char dataByte) {
    unsigned char startCounter = 100;


    // I2C START BIT
    while (startCounter--) {
        if (StartTransfer(FALSE))break;
        DelayMs(10);
    }

    if (!startCounter)return (FALSE);


    // Write device ID
    if (!TransmitOneByte((deviceID << 1) | I2C_WRITE))return (FALSE);

    // Verify that the byte was acknowledged
    if (!I2CByteWasAcknowledged(EEPROM_I2C_BUS))return (FALSE);

    // Write first register to be addressed
    if (!TransmitOneByte(deviceREGISTER))return (FALSE);

    // Verify that the byte was acknowledged
    if (!I2CByteWasAcknowledged(EEPROM_I2C_BUS))return (FALSE);

    // Write data byte
    if (!TransmitOneByte(dataByte))return (FALSE);

    // Verify that the byte was acknowledged
    if (!I2CByteWasAcknowledged(EEPROM_I2C_BUS))return (FALSE);

    // End of I2C transaction
    I2CStop(EEPROM_I2C_BUS);

    return (TRUE);

}

unsigned char readRegisters(unsigned char deviceID, unsigned char deviceREGISTER, unsigned char numRegisters, unsigned char *registerPtr) {
    unsigned char i;

    if (numRegisters >= MAXBUFFER)return (0);
    if (registerPtr == NULL)return (0);

    if (!setRegister(deviceID, deviceREGISTER)) return (FALSE);
    if (!sendREADcommand(deviceID)) return (FALSE);

    for (i = 0; i < numRegisters; i++) {
        if (I2CReceiverEnable(EEPROM_I2C_BUS, TRUE) == I2C_RECEIVE_OVERFLOW) // Enable the module to receive data from the I2C bus.
            return (0);
        else {
            short timeout = 1000;
            while (!I2CReceivedDataIsAvailable(EEPROM_I2C_BUS)) { // Wait for data from slave
                timeout--;
                if (!timeout) return (0);
                DelayMs(1);
            }

            if (i < (numRegisters - 1)) I2CAcknowledgeByte(EEPROM_I2C_BUS, TRUE); // Set ACK or NAK flag appropriately
            else I2CAcknowledgeByte(EEPROM_I2C_BUS, FALSE);
            registerPtr[i] = I2CGetByte(EEPROM_I2C_BUS); // Read a byte from slave

            timeout = 1000;
            while (!I2CAcknowledgeHasCompleted(EEPROM_I2C_BUS)) { // Wait for acknowledge bit to be sent
                timeout--;
                if (!timeout) return (0);
                DelayMs(1);
            }
        }
    }

    I2CStop(EEPROM_I2C_BUS);

    return (i);
}

unsigned char initMMA8452(void) {
    unsigned char accelData[4] = {0, 0, 0, 0};
    unsigned char commandByte;

    DelayMs(10);

    if (!readRegisters(ACCELEROMETER_ID, WHO_AM_I, 1, accelData)) return (FALSE); // Read WHO_AM_I register
    if (accelData[0] != 0x2A) return (FALSE); // WHO_AM_I should always be 0x2A
    DelayMs(10);

    if (!readRegisters(ACCELEROMETER_ID, CTRL_REG1, 1, accelData)) return (FALSE); // Put in READY mode
    commandByte = accelData[0];
    commandByte = commandByte & 0xFE;
    DelayMs(10);
    if (!writeByteToRegister(ACCELEROMETER_ID, CTRL_REG1, commandByte)) return (FALSE);

    commandByte = GSCALE;
    if (commandByte > 8) commandByte = 8;
    commandByte = commandByte >> 2;
    DelayMs(10);
    if (!writeByteToRegister(ACCELEROMETER_ID, XYZ_DATA_CFG, commandByte)) return (FALSE);


    DelayMs(10);
    if (!readRegisters(ACCELEROMETER_ID, CTRL_REG1, 1, accelData)) return (FALSE); // Put in ACTIVE mode
    commandByte = accelData[0];
    commandByte = commandByte | 0x01;

    DelayMs(10);
    if (!writeByteToRegister(ACCELEROMETER_ID, CTRL_REG1, commandByte)) return (FALSE);

    return (TRUE);
}


short convertValue(unsigned char MSBbyte, unsigned char LSBbyte) {
    short value;

    // Combine high and low bytes
    value = MSBbyte;
    value = value << 8;
    value = value | LSBbyte;

    //The registers are left align, here we right align the 12-bit integer
    value = value >> 4;

    // If the number is negative, we have to make it so manually (no 12-bit data type)
    if ((MSBbyte & 0x80) != 0) {
        value = ~value + 1;
        value = value * -1; // Transform into negative 2's complement #
    }
    return (value);
}

/*******************************************************************************
  Function:
    void StopTransfer( void )

  Summary:
    Stops a transfer to/from the EEPROM.

  Description:
    This routine Stops a transfer to/from the EEPROM, waiting (in a
    blocking loop) until the Stop condition has completed.

  Precondition:
    The I2C module must have been initialized & a transfer started.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    StopTransfer();
    </code>

  Remarks:
    This is a blocking routine that waits for the Stop signal to complete.
 *****************************************************************************/

void StopTransfer(void) {
    I2C_STATUS status;

    // Send the Stop signal
    I2CStop(EEPROM_I2C_BUS);

    // Wait for the signal to complete
    do {
        status = I2CGetStatus(EEPROM_I2C_BUS);

    } while (!(status & I2C_STOP));
}
