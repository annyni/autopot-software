// This #include statement was automatically added by the Particle IDE.
//#include "SparkIntervalTimer/SparkIntervalTimer.h"
#include "application.h"
#include "FT800.h"				// FT800 register, memory and command values
//#include "DS18B20/DS18B20.h"

// Delcarations
// Also see FT800_hw.h and FT800_sw.h
//
// Set LCD display resolution here
//#define LCD_QVGA				// QVGA  = 320 x 240 (VM800B/C 3.5")
#define LCD_WQVGA				// WQVGA = 480 x 272 (VM800B/C 4.3" and 5.0")

// Set Arduino platform here
//#define VM800B				// FTDI FT800 "Plus" board with AT328P (I/O 9 on SS#)
#define ARDUINO					// Arduino Pro, Uno, etc. (I/O 10 on SS#)

// FT800 Chip Commands - use with cmdWrite//
#define FT800_ACTIVE	0x00			// Initializes FT800
#define FT800_STANDBY	0x41			// Place FT800 in Standby (clk running)
#define FT800_SLEEP	    0x42			// Place FT800 in Sleep (clk off)
#define FT800_PWRDOWN	0x50			// Place FT800 in Power Down (core off)
#define FT800_CLKEXT	0x44			// Select external clock source
#define FT800_CLK48M	0x62			// Select 48MHz PLL
#define FT800_CLK36M	0x61			// Select 36MHz PLL
#define FT800_CORERST	0x68			// Reset core - all registers default

// FT800 Memory Commands - use with ft800memWritexx and ft800memReadxx
#define MEM_WRITE	0x80			// FT800 Host Memory Write
#define MEM_READ	0x00			// FT800 Host Memory Read

// Colors - fully saturated colors defined here
#define RED	    	0xFF0000UL		// Red
#define GREEN		0x00FF00UL		// Green
#define BLUE		0x0000FFUL		// Blue
#define WHITE		0xFFFFFFUL		// White
#define BLACK		0x000000UL		// Black


//SYSTEM_MODE(MANUAL);

//IntervalTimer myTimer;
//DS18B20 ds18b20 = DS18B20(D2);
//unsigned int DS18B20nextSampleTime;
//unsigned int DS18B20_SAMPLE_INTERVAL = 600;
//unsigned int nextUpdateTime;
//unsigned int UPDATE_INTERVAL = 300;
//double celsius;
//double fahrenheit;
//char pubString[15];
// Global Variables

// Arduino pins - others defined by Serial and SPI libraries

// LCD display parameters
unsigned int lcdWidth = 480;				// Active width of LCD display
unsigned int lcdHeight = 272;				// Active height of LCD display
unsigned int lcdHcycle = 548;				// Total number of clocks per line
unsigned int lcdHoffset = 43;			// Start of active line
unsigned int lcdHsync0 = 0;				// Start of horizontal sync pulse
unsigned int lcdHsync1 = 41;				// End of horizontal sync pulse
unsigned int lcdVcycle = 292;				// Total number of lines per screen
unsigned int lcdVoffset = 12;			// Start of active screen
unsigned int lcdVsync0 = 0;				// Start of vertical sync pulse
unsigned int lcdVsync1 = 10;				// End of vertical sync pulse
unsigned char lcdPclk = 5;				// Pixel Clock
unsigned char lcdSwizzle = 0;			// Define RGB output pins
unsigned char lcdPclkpol = 1;			// Define active edge of PCLK

unsigned long ramDisplayList = RAM_DL;		// Set beginning of display list memory
unsigned long ramCommandBuffer = RAM_CMD;	// Set beginning of graphics command memory
/*
unsigned int cmdBufferRd = 0x0000;		// Used to navigate command ring buffer
unsigned int cmdBufferWr = 0x0000;		// Used to navigate command ring buffer
unsigned int cmdOffset = 0x0000;		// Used to navigate command rung buffer
unsigned int point_size = 20;		// Define a default dot size
unsigned long point_x = (96 * 16);		// Define a default point x-location (1/16 anti-aliased)
unsigned long point_y = (136 * 16);		// Define a default point y-location (1/16 anti-aliased)
unsigned long color;				// Variable for chanign colors
*/
unsigned char ft800Gpio;			// Used for FT800 GPIO register


FT800::FT800(){
  ft800irqPin = A2;				// Interrupt from FT800 to Arduino - not used here
  ft800pwrPin = A0;				// PD_N from Arduino to FT800 - effectively FT800 reset
  ft800csPin  = DAC;				// SPI chip select - defined separately since it's manipulated with GPIO calls

}

/******************************************************************************
 * Function:        void ft800memWritexx(ftAddress, ftDataxx, ftLength)
 * PreCondition:    None
 * Input:           ftAddress = FT800 memory space address
 *                  ftDataxx = a byte, int or long to send
 * Output:          None
 * Side Effects:    None
 * Overview:        Writes FT800 internal address space
 * Note:            "xx" is one of 8, 16 or 32
 *****************************************************************************/
void FT800::ft800memWrite8(unsigned long ftAddress, unsigned char ftData8)
{
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer((char)(ftAddress >> 16) | MEM_WRITE); // Send Memory Write plus high address byte
  SPI.transfer((char)(ftAddress >> 8));		// Send middle address byte
  SPI.transfer((char)(ftAddress));		// Send low address byte
  SPI.transfer(ftData8);			// Send data byte
  digitalWrite(ft800csPin, HIGH);		// Set CS# high
}

void FT800::ft800memWrite16(unsigned long ftAddress, unsigned int ftData16)
{
//  digitalWrite(triggerPin, HIGH);		// Toggle a pin to trigger the oscilloscope
//  digitalWrite(triggerPin, LOW);		// Toggle a pin to trigger the oscilloscope
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer((char)(ftAddress >> 16) | MEM_WRITE); // Send Memory Write plus high address byte
  SPI.transfer((char)(ftAddress >> 8));		// Send middle address byte
  SPI.transfer((char)(ftAddress));		// Send low address byte
  SPI.transfer((char)(ftData16));		// Send data low byte
  SPI.transfer((char)(ftData16 >> 8));		// Send data high byte
  digitalWrite(ft800csPin, HIGH);		// Set CS# high
}

void FT800::ft800memWrite32(unsigned long ftAddress, unsigned long ftData32)
{
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer((char)(ftAddress >> 16) | MEM_WRITE); // Send Memory Write plus high address byte
  SPI.transfer((char)(ftAddress >> 8));		// Send middle address byte
  SPI.transfer((char)(ftAddress));		// Send low address byte
  SPI.transfer((char)(ftData32));		// Send data low byte
  SPI.transfer((char)(ftData32 >> 8));
  SPI.transfer((char)(ftData32 >> 16));
  SPI.transfer((char)(ftData32 >> 24));		// Send data high byte
  digitalWrite(ft800csPin, HIGH);		// Set CS# high
}
/******************************************************************************
 * Function:        unsigned char ft800memReadxx(ftAddress, ftLength)
 * PreCondition:    None
 * Input:           ftAddress = FT800 memory space address
 * Output:          ftDataxx (byte, int or long)
 * Side Effects:    None
 * Overview:        Reads FT800 internal address space
 * Note:            "xx" is one of 8, 16 or 32
 *****************************************************************************/
unsigned char FT800::ft800memRead8(unsigned long ftAddress)
{
  unsigned char ftData8 = ZERO;
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer((char)(ftAddress >> 16) | MEM_READ); // Send Memory Write plus high address byte
  SPI.transfer((char)(ftAddress >> 8));		// Send middle address byte
  SPI.transfer((char)(ftAddress));		// Send low address byte
  SPI.transfer(ZERO);				// Send dummy byte
    ftData8 = SPI.transfer(ZERO);		// Read data byte
  digitalWrite(ft800csPin, HIGH);		// Set CS# high
  return ftData8;				// Return byte read
}

unsigned int FT800::ft800memRead16(unsigned long ftAddress)
{
  unsigned int ftData16;
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer((char)(ftAddress >> 16) | MEM_READ); // Send Memory Write plus high address byte
  SPI.transfer((char)(ftAddress >> 8));		// Send middle address byte
  SPI.transfer((char)(ftAddress));		// Send low address byte
  SPI.transfer(ZERO);				// Send dummy byte
    ftData16 = (SPI.transfer(ZERO));		// Read low byte
    ftData16 = (SPI.transfer(ZERO) << 8) | ftData16; // Read high byte
  digitalWrite(ft800csPin, HIGH);		// Set CS#hHigh
  return ftData16;				// Return integer read
}

unsigned long FT800::ft800memRead32(unsigned long ftAddress)
{
  unsigned long ftData32;
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer((char)(ftAddress >> 16) | MEM_READ); // Send Memory Write plus high address byte
  SPI.transfer((char)(ftAddress >> 8));		// Send middle address byte
  SPI.transfer((char)(ftAddress));		// Send low address byte
  SPI.transfer(ZERO);				// Send dummy byte
    ftData32 = (SPI.transfer(ZERO));		// Read low byte
    ftData32 = (SPI.transfer(ZERO) << 8) | ftData32;
    ftData32 = (SPI.transfer(ZERO) << 16) | ftData32;
    ftData32 = (SPI.transfer(ZERO) << 24) | ftData32; // Read high byte
  digitalWrite(ft800csPin, HIGH);		// Set CS# high
  return ftData32;				// Return long read
}
/******************************************************************************
 * Function:        void ft800cmdWrite(ftCommand)
 * PreCondition:    None
 * Input:           None
 * Output:          None
 * Side Effects:    None
 * Overview:        Sends FT800 command
 * Note:            None
 *****************************************************************************/
void FT800::ft800cmdWrite(unsigned char ftCommand)
{
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer(ftCommand);			// Send command
  SPI.transfer(0x00);				// Commands consist of two more zero bytes
  SPI.transfer(0x00);				// Send last zero byte
  digitalWrite(ft800csPin, HIGH);		// Set CS# high
}

/******************************************************************************
 * Function:        void incCMDOffset(currentOffset, commandSize)
 * PreCondition:    None
 *                    starting a command list
 * Input:           currentOffset = graphics processor command list pointer
 *                  commandSize = number of bytes to increment the offset
 * Output:          newOffset = the new ring buffer pointer after adding the command
 * Side Effects:    None
 * Overview:        Adds commandSize to the currentOffset.
 *                  Checks for 4K ring-buffer offset roll-over
 * Note:            None
 *****************************************************************************/
unsigned int FT800::incCMDOffset(unsigned int currentOffset, unsigned char commandSize)
{
    unsigned int newOffset;			// used to hold new offset
    newOffset = currentOffset + commandSize;	// Calculate new offset
    if(newOffset > 4095)			// If new offset past boundary...
    {
        newOffset = (newOffset - 4096);		// ... roll over pointer
    }
    return newOffset;				// Return new offset
}

unsigned int FT800::displayText(FT800 ft800, unsigned long ftAddress, const char* text, unsigned int initX, unsigned int initY, unsigned int cmdOffset)
{
  int len = strlen(text);
  for (int i = 0; i < len; i++)
  {
    char c = text[i];
    ft800memWrite32(ftAddress + cmdOffset, (CMD_TEXT));
    cmdOffset = ft800.incCMDOffset(cmdOffset, 4);

    ft800.ft800memWrite32(ftAddress + cmdOffset, (initX+20));
    cmdOffset = ft800.incCMDOffset(cmdOffset, 2);

    ft800.ft800memWrite32(ftAddress + cmdOffset, (initY));
    cmdOffset = ft800.incCMDOffset(cmdOffset, 2);

    ft800.ft800memWrite32(ftAddress + cmdOffset, (31));
    cmdOffset = ft800.incCMDOffset(cmdOffset, 2);	// Update the command pointer

    ft800.ft800memWrite32(ftAddress + cmdOffset, (OPT_CENTER));
    cmdOffset = ft800.incCMDOffset(cmdOffset, 2);	// Update the command pointer

    ft800.ft800memWrite8(ftAddress + cmdOffset, c);
    cmdOffset = ft800.incCMDOffset(cmdOffset, 4);
    initX += 20;
  }
  return cmdOffset;
}

void FT800::init(FT800 ft800) {
  //ft800.triggerPin = 2;				// Used for oscilloscope/logic analyzer trigger
/*  ft800.ft800irqPin = A2;				// Interrupt from FT800 to Arduino - not used here
  ft800.ft800pwrPin = A0;				// PD_N from Arduino to FT800 - effectively FT800 reset
  ft800.ft800csPin  = DAC;				// SPI chip select - defined separately since it's manipulated with GPIO calls

  //pinMode(ft800.triggerPin, OUTPUT);			// Arduino pin used for oscilloscope triggering
  pinMode(ft800.ft800irqPin, INPUT_PULLUP);		// FT800 interrupt output (not used in this example)
  pinMode(ft800.ft800pwrPin, OUTPUT);			// FT800 Power Down (reset) input
  pinMode(ft800.ft800csPin, OUTPUT);			// FT800 SPI bus CS# input

  //digitalWrite(ft800.triggerPin, LOW);		// Initialize the oscilloscope trigger
  digitalWrite(ft800.ft800csPin, HIGH);		// Set CS# high to start - SPI inactive
  digitalWrite(ft800.ft800pwrPin, HIGH);		// Set PD# high to start
  delay(20);					// Wait a few MS before waking the FT800

  //***************************************
// Wake-up FT800

  digitalWrite(ft800.ft800pwrPin, LOW);		// 1) lower PD#
  delay(20);					// 2) hold for 20ms
  digitalWrite(ft800.ft800pwrPin, HIGH);		// 3) raise PD#
  delay(20);					// 4) wait for another 20ms before sending any commands
*/
  ft800.ft800cmdWrite(FT800_ACTIVE);			// Start FT800
  delay(5);					// Give some time to process
  ft800.ft800cmdWrite(FT800_CLKEXT);			// Set FT800 for external clock
  delay(5);					// Give some time to process
  ft800.ft800cmdWrite(FT800_CLK48M);			// Set FT800 for 48MHz PLL
  delay(5);					// Give some time to process
              // Now FT800 can accept commands at up to 30MHz clock on SPI bus
              //   This application leaves the SPI bus at 4MHz

  if (ft800.ft800memRead8(REG_ID) != 0x7C)		// Read ID register - is it 0x7C?
  {
      Serial.write("System Halted!\r\n");		// Send an error message on the UART
      while(1);					// If we don't get 0x7C, the ineface isn't working - halt with infinite loop
  }

  ft800.ft800memWrite8(REG_PCLK, ZERO);		// Set PCLK to zero - don't clock the LCD until later
  ft800.ft800memWrite8(REG_PWM_DUTY, ZERO);		// Turn off backlight

  // End of Wake-up FT800
  //***************************************

  //***************************************
  // Initialize Display
  ft800.ft800memWrite16(REG_HSIZE,   lcdWidth);	// active display width
  ft800.ft800memWrite16(REG_HCYCLE,  lcdHcycle);	// total number of clocks per line, incl front/back porch
  ft800.ft800memWrite16(REG_HOFFSET, lcdHoffset);	// start of active line
  ft800.ft800memWrite16(REG_HSYNC0,  lcdHsync0);	// start of horizontal sync pulse
  ft800.ft800memWrite16(REG_HSYNC1,  lcdHsync1);	// end of horizontal sync pulse
  ft800.ft800memWrite16(REG_VSIZE,   lcdHeight);	// active display height
  ft800.ft800memWrite16(REG_VCYCLE,  lcdVcycle);	// total number of lines per screen, incl pre/post
  ft800.ft800memWrite16(REG_VOFFSET, lcdVoffset);	// start of active screen
  ft800.ft800memWrite16(REG_VSYNC0,  lcdVsync0);	// start of vertical sync pulse
  ft800.ft800memWrite16(REG_VSYNC1,  lcdVsync1);	// end of vertical sync pulse
  ft800.ft800memWrite8(REG_SWIZZLE,  lcdSwizzle);	// FT800 output to LCD - pin order
  ft800.ft800memWrite8(REG_PCLK_POL, lcdPclkpol);	// LCD data is clocked in on this PCLK edge
  ft800.ft800memWrite32(REG_ROTATE, 0);
          // Don't set PCLK yet - wait for just after the first display list
          // End of Initialize Display
          //***************************************

          //***************************************
          // Configure Touch and Audio - not used in this example, so disable both
  ft800.ft800memWrite8(REG_TOUCH_MODE, ZERO);		// Disable touch
  ft800.ft800memWrite16(REG_TOUCH_RZTHRESH, ZERO);	// Eliminate any false touches

  ft800.ft800memWrite8(REG_VOL_PB, ZERO);		// turn recorded audio volume down
  ft800.ft800memWrite8(REG_VOL_SOUND, ZERO);		// turn synthesizer volume down
  ft800.ft800memWrite16(REG_SOUND, 0x6000);		// set synthesizer to mute

  // End of Configure Touch and Audio
  //***************************************

  //***************************************
  // Write Initial Display List & Enable Display

  ramDisplayList = RAM_DL;			// start of Display List
  ft800.ft800memWrite32(ramDisplayList, DL_CLEAR_RGB); // Clear Color RGB   00000010 RRRRRRRR GGGGGGGG BBBBBBBB  (R/G/B = Colour values) default zero / black
  ramDisplayList += 4;				// point to next location
  ft800.ft800memWrite32(ramDisplayList, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));	// Clear 00100110 -------- -------- -----CST  (C/S/T define which parameters to clear)
  ramDisplayList += 4;				// point to next location
  ft800.ft800memWrite32(ramDisplayList, DL_DISPLAY);	// DISPLAY command 00000000 00000000 00000000 00000000 (end of display list)

  ft800.ft800memWrite32(REG_DLSWAP, DLSWAP_FRAME);	// 00000000 00000000 00000000 000000SS  (SS bits define when render occurs)
          // Nothing is being displayed yet... the pixel clock is still 0x00
  ramDisplayList = RAM_DL;			// Reset Display List pointer for next list

  ft800Gpio = ft800.ft800memRead8(REG_GPIO);		// Read the FT800 GPIO register for a read/modify/write operation
  ft800Gpio = ft800Gpio | 0x80;			// set bit 7 of FT800 GPIO register (DISP) - others are inputs
  ft800.ft800memWrite8(REG_GPIO, ft800Gpio);		// Enable the DISP signal to the LCD panel
  ft800.ft800memWrite8(REG_PCLK, lcdPclk);		// Now start clocking data to the LCD panel
  for(int duty = 0; duty <= 128; duty++)
  {
    ft800.ft800memWrite8(REG_PWM_DUTY, duty);		// Turn on backlight - ramp up slowly to full brighness
    delay(10);
  }
  // End of Write Initial Display List & Enable Display
  //***************************************
}

void FT800::draw(FT800 ft800, unsigned int cmdOffset, unsigned int COLOR, double celsius, unsigned int plant_temp_min, unsigned int plant_temp_max)
{
  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (CMD_DLSTART));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR_RGB | COLOR));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_SCISSOR_SIZE | 480<<12 | 272));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR_RGB | COLOR));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (CMD_NUMBER));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (20));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 2);

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (90));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 2);

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (31));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 2);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (OPT_CENTERY));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 2);	// Update the command pointer

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, ((int)(celsius)));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  if (celsius < plant_temp_min) {
    cmdOffset = ft800.displayText(ft800, RAM_CMD, "i'm cold! move me! >.<", 10, 145, cmdOffset);
  } else if (celsius > plant_temp_max) {
    cmdOffset = ft800.displayText(ft800, RAM_CMD, "it's too hot! help! ~_~", 10, 145, cmdOffset);
  } else {
    cmdOffset = ft800.displayText(ft800, RAM_CMD, "i'm happy! ^_^", 10, 145, cmdOffset);
  }

  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_DISPLAY));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer


  ft800.ft800memWrite32(RAM_CMD + cmdOffset, (CMD_SWAP));
  cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800.ft800memWrite16(REG_CMD_WRITE, (cmdOffset));
}
