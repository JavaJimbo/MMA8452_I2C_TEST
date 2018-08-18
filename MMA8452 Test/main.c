/***************************************************************************************
 * Project:     MMA8452 Test
 * For PIC32MX220 on Two Motor board V2
 * Compiler: XC32 V1.30
 * FileName:    main.c 
 * 
 * 8-18-18: Works with Sparkfun MMA8452 breakout board, ACCELEROMETER_ID = 0x1D, I2C Bus #1 
 * Comment out TEST_MMA8452 to test 24LC256 EEPROM @ 0xA0
 ****************************************************************************************/

#define SYS_FREQ 60000000
#define GetPeripheralClock() SYS_FREQ 

#define TEST_MMA8452

#include "I2C_EEPROM_PIC32.h"
#include "MMA8452.h"
#include "PCA9685.h"
#include "Delay.h"
#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/** CONFIGURATION **************************************************/
#pragma config UPLLEN   = ON            // USB PLL Enabled
#pragma config FPLLMUL  = MUL_15        // PLL Multiplier for 220 - yields 60 Mhz
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
#pragma config FPLLODIV = DIV_1         // PLL Output Divider
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = HS            // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = PRIPLL        // Oscillator Selection
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = OFF           // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // Use JTAG pins for normal IO
#pragma config DEBUG    = OFF            // Enable/disable debugging


/*** DEFINES *****************************************************************/
#define HOSTuart UART2
#define SYS_FREQ 60000000  // With 8 Mhz crystal and FPLLMUL = MUL_15
#define HOSTbits U2STAbits
#define HOST_VECTOR _UART_2_VECTOR     
#define MAXBUFFER 64
#define false FALSE
#define true TRUE
#define PCA9685_ADDRESS 0x80 // Basic default address for Adafruit Feather Servo Board   

/** V A R I A B L E S ********************************************************/
#define MAXDATABYTES 64
unsigned char arrData[MAXDATABYTES];
unsigned char HOSTRxBuffer[MAXBUFFER+1];
unsigned char HOSTTxBuffer[MAXBUFFER+1];
unsigned char HOSTRxBufferFull = FALSE;

#define MAXPOTS 1
unsigned short arrPots[MAXPOTS];
unsigned char dataReady = false;

/** P R O T O T Y P E S ***************************************/
void InitializeSystem(void);
void ConfigAd(void);
unsigned char setPCA9685outputs(unsigned char device, unsigned short channel, unsigned short turnON, unsigned short turnOFF);
unsigned char initializePCA9685(unsigned char device);
// void putch(unsigned char ch);

void ConfigAd(void) {

    mPORTCSetPinsAnalogIn(BIT_1);

    // ---- configure and enable the ADC ----

    // ensure the ADC is off before setting the configuration
    CloseADC10();

    // define setup parameters for OpenADC10
    //                 Turn module on | ouput in integer | trigger mode auto | enable autosample
#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

    // ADC ref external    | disable offset test    | enable scan mode | perform  samples | use dual buffers | use only mux A
#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_ON | ADC_ALT_INPUT_OFF

    //                   use ADC internal clock | set sample time
#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_31

//  set AN7 (A1 on Olimex 220 board) input to analog
// #define PARAM4    ENABLE_AN0_ANA | ENABLE_AN1_ANA| ENABLE_AN2_ANA | ENABLE_AN3_ANA
#define PARAM4    ENABLE_AN7_ANA


    // USE AN7    
#define PARAM5 SKIP_SCAN_AN0 |SKIP_SCAN_AN1 |SKIP_SCAN_AN2 |SKIP_SCAN_AN3 |\
SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN6 |\
SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 |\
SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15


    // set negative reference to Vref for Mux A
    SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    // open the ADC
    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    ConfigIntADC10(ADC_INT_PRI_2 | ADC_INT_SUB_PRI_2 | ADC_INT_ON);

    // clear the interrupt flag
    mAD1ClearIntFlag();

    // Enable the ADC
    EnableADC10();
}

void __ISR(_ADC_VECTOR, ipl6) ADHandler(void) {
    unsigned short offSet;
    unsigned char i;

    mAD1IntEnable(INT_DISABLED);
    mAD1ClearIntFlag();

    // Determine which buffer is idle and create an offset
    offSet = 8 * ((~ReadActiveBufferADC10() & 0x01));

    for (i = 0; i < MAXPOTS; i++)
        arrPots[i] = (unsigned short) ReadADC10(offSet + i); // read the result of channel 0 conversion from the idle buffer
    dataReady = TRUE;
}


void __ISR(HOST_VECTOR, ipl2) IntHostUartHandler(void) {
    static unsigned int RxIndex = 0;
    unsigned char ch;

    if (INTGetFlag(INT_SOURCE_UART_RX(HOSTuart))) {
        INTClearFlag(INT_SOURCE_UART_RX(HOSTuart));
        if (HOSTbits.OERR || HOSTbits.FERR) {
            if (UARTReceivedDataIsAvailable(HOSTuart))
                ch = UARTGetDataByte(HOSTuart);
            HOSTbits.OERR = 0;
            RxIndex = 0;
        }

        if (UARTReceivedDataIsAvailable(HOSTuart)) {
            ch = toupper(UARTGetDataByte(HOSTuart));
            if (RxIndex < MAXBUFFER) HOSTRxBuffer[RxIndex++] = ch;
            if (ch == '\r')
            {
                HOSTRxBufferFull = TRUE;
                HOSTRxBuffer[RxIndex] = '\0';
                RxIndex = 0;
            }
        }

        if (INTGetFlag(INT_SOURCE_UART_TX(HOSTuart))) {
            INTClearFlag(INT_SOURCE_UART_TX(HOSTuart));
        }
    }
}

/*
void putch(unsigned char ch) {
    while (!IFS1bits.U2TXIF); // set when register is empty 
    U2TXREG = ch;
}
*/

void InitializeSystem(void) {    

    mJTAGPortEnable(false);
    PORTSetPinsDigitalOut(IOPORT_C, BIT_5 | BIT_7);
    PORTSetPinsDigitalOut(IOPORT_A, BIT_7 | BIT_10);    
    PORTSetPinsDigitalOut(IOPORT_B, BIT_4);  
         
    
    PORTSetPinsDigitalIn(IOPORT_C, BIT_1 | BIT_9);    
    
    ANSELBbits.ANSB15 = 0;
    ANSELBbits.ANSB14 = 0;
    ANSELBbits.ANSB13 = 0;
      
    PORTSetPinsDigitalIn(IOPORT_B, BIT_13 | BIT_14 | BIT_15);

    // Set up main UART    
    PPSOutput(4, RPC2, U2TX);
    PPSInput(2, U2RX, RPA8);
    
    UARTConfigure(HOSTuart, UART_ENABLE_HIGH_SPEED | UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(HOSTuart, UART_INTERRUPT_ON_RX_NOT_EMPTY); //  | UART_INTERRUPT_ON_TX_DONE  
    UARTSetLineControl(HOSTuart, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(HOSTuart, SYS_FREQ, 57600);
    UARTEnable(HOSTuart, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    
    // Configure UART #2 Interrupts
    INTEnable(INT_U2TX, INT_DISABLED);
    INTEnable(INT_U2RX, INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(HOSTuart), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(HOSTuart), INT_SUB_PRIORITY_LEVEL_0);

    ConfigAd();

    // Turn on the interrupts
    INTEnableSystemMultiVectoredInt();
}//end UserInit

#define MIN_CHANGE 16
int main(void) 
{
    int length = 0;
    short rawVectx, rawVecty, rawVectz;
    short PreviousX, PreviousY, PreviousZ;
    unsigned char accelerometerBuffer[MAX_I2C_REGISTERS];
    
    InitializeSystem();
    DelayMs(100);
    
#ifdef TEST_MMA8452    
    printf("\r\rOpening I2C #1...");
    OpenI2C(I2C_EN, 299);    
    printf("\rInitializing MMA8452...");
    if (initMMA8452()) printf("SUCCESS!");
    else
    {
        printf("ERROR: failed init");
        while(1);
    }
    printf("\rMIN CHNAGE = %d", MIN_CHANGE);
    DelayMs(100);
        
    printf("\rTesting MMA8452, no interrupts");
        
    while(1)
    {                 
        DelayMs(1);
        readRegisters(ACCELEROMETER_ID, 0x01, MAX_I2C_REGISTERS, accelerometerBuffer);
        rawVectx = convertValue(accelerometerBuffer[0], accelerometerBuffer[1]) / 8;
        rawVectz = convertValue(accelerometerBuffer[2], accelerometerBuffer[3]) / 8;
        rawVecty = convertValue(accelerometerBuffer[4], accelerometerBuffer[5]) / 8;
        
        if (abs(PreviousX - rawVectx) > MIN_CHANGE || abs(PreviousY - rawVecty) > MIN_CHANGE || abs(PreviousZ - rawVectz) > MIN_CHANGE)
        {
            printf("\rX: %d, Y: %d, Z: %d", rawVectx, rawVecty, rawVectz);
            PreviousX = rawVectx;
            PreviousY = rawVecty;
            PreviousZ = rawVectz;
        }
    }
}
#else
    printf("\r\rOpening I2C #1...");
    OpenI2C(I2C_EN, 299);    
    printf("\rTesting EEPROM @ %X", EEPROM_ADDRESS);
        
    while(1)
    {
        if (HOSTRxBufferFull)
        {
            HOSTRxBufferFull = false;
            printf("\rWriting: %s", HOSTRxBuffer);
            length = strlen(HOSTRxBuffer);
            EepromWriteBlock(EEPROM_ADDRESS, 0x0000, HOSTRxBuffer, length);
            DelayMs(200);
            EepromReadBlock(EEPROM_ADDRESS, 0x0000, HOSTTxBuffer, length);
            printf("\rReading: %s", HOSTRxBuffer);
        }
        DelayMs(1);
    } // end while(1)
}        
#endif        
  

