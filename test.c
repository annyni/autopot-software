// This #include statement was automatically added by the Particle IDE.
//#include "SparkIntervalTimer/SparkIntervalTimer.h"

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

IntervalTimer myTimer;
//DS18B20 ds18b20 = DS18B20(D2);
unsigned int DS18B20nextSampleTime;
unsigned int DS18B20_SAMPLE_INTERVAL = 600;
unsigned int nextUpdateTime;
unsigned int UPDATE_INTERVAL = 300;
double celsius;
double fahrenheit;
char pubString[15];
unsigned int level=29;
unsigned int COLOR=0x0091D3;
unsigned int color_target=0x0091D3;
unsigned int plant_temp_max= 38;
unsigned int plant_temp_min= 23;
unsigned int shower_time=30;
unsigned int shower_temp=38;
// Global Variables

// Arduino pins - others defined by Serial and SPI libraries
unsigned int triggerPin;			// Used for oscilloscope/logic analyzer trigger
unsigned int ft800irqPin;			// Interrupt from FT800 to Arduino - not used here
unsigned int ft800pwrPin;			// PD_N from Arduino to FT800 - effectively FT800 reset
unsigned int ft800csPin;			// SPI chip select - defined separately since it's manipulated with GPIO calls

// LCD display parameters
unsigned int lcdWidth;				// Active width of LCD display
unsigned int lcdHeight;				// Active height of LCD display
unsigned int lcdHcycle;				// Total number of clocks per line
unsigned int lcdHoffset;			// Start of active line
unsigned int lcdHsync0;				// Start of horizontal sync pulse
unsigned int lcdHsync1;				// End of horizontal sync pulse
unsigned int lcdVcycle;				// Total number of lines per screen
unsigned int lcdVoffset;			// Start of active screen
unsigned int lcdVsync0;				// Start of vertical sync pulse
unsigned int lcdVsync1;				// End of vertical sync pulse
unsigned char lcdPclk;				// Pixel Clock
unsigned char lcdSwizzle;			// Define RGB output pins
unsigned char lcdPclkpol;			// Define active edge of PCLK

unsigned long ramDisplayList = RAM_DL;		// Set beginning of display list memory
unsigned long ramCommandBuffer = RAM_CMD;	// Set beginning of graphics command memory

unsigned int cmdBufferRd = 0x0000;		// Used to navigate command ring buffer
unsigned int cmdBufferWr = 0x0000;		// Used to navigate command ring buffer
unsigned int cmdOffset = 0x0000;		// Used to navigate command rung buffer
unsigned int point_size = 20;		// Define a default dot size
unsigned long point_x = (96 * 16);		// Define a default point x-location (1/16 anti-aliased)
unsigned long point_y = (136 * 16);		// Define a default point y-location (1/16 anti-aliased)
unsigned long color;				// Variable for chanign colors
unsigned char ft800Gpio;			// Used for FT800 GPIO register

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
void ft800memWrite8(unsigned long ftAddress, unsigned char ftData8)
{
  digitalWrite(ft800csPin, LOW);		// Set CS# low
  SPI.transfer((char)(ftAddress >> 16) | MEM_WRITE); // Send Memory Write plus high address byte
  SPI.transfer((char)(ftAddress >> 8));		// Send middle address byte
  SPI.transfer((char)(ftAddress));		// Send low address byte
  SPI.transfer(ftData8);			// Send data byte
  digitalWrite(ft800csPin, HIGH);		// Set CS# high
}

void ft800memWrite16(unsigned long ftAddress, unsigned int ftData16)
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

void ft800memWrite32(unsigned long ftAddress, unsigned long ftData32)
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
unsigned char ft800memRead8(unsigned long ftAddress)
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

unsigned int ft800memRead16(unsigned long ftAddress)
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

unsigned long ft800memRead32(unsigned long ftAddress)
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
void ft800cmdWrite(unsigned char ftCommand)
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
unsigned int incCMDOffset(unsigned int currentOffset, unsigned char commandSize)
{
    unsigned int newOffset;			// used to hold new offset
    newOffset = currentOffset + commandSize;	// Calculate new offset
    if(newOffset > 4095)			// If new offset past boundary...
    {
        newOffset = (newOffset - 4096);		// ... roll over pointer
    }
    return newOffset;				// Return new offset
}

/*void getTemp(){
    if(!ds18b20.search()){
      ds18b20.resetsearch();
      celsius = ds18b20.getTemperature();
      fahrenheit = ds18b20.convertToFahrenheit(celsius);
      DS18B20nextSampleTime = millis() + DS18B20_SAMPLE_INTERVAL;
      //Serial.println(fahrenheit);
    }
}*/

void setup() {
    pinMode(P1S4, OUTPUT);
    digitalWrite(P1S4,LOW);
    Time.zone(-4);
    //DS18B20nextSampleTime=millis();
    nextUpdateTime=millis();
    Particle.syncTime();
    pinMode(D2, INPUT);
    // need change
    triggerPin = A0;				// Used for oscilloscope/logic analyzer trigger
    ft800irqPin = D1;				// Interrupt from FT800 to Arduino - not used here
    ft800pwrPin = D0;				// PD_N from Arduino to FT800 - effectively FT800 reset
    ft800csPin  = DAC;				// SPI chip select - defined separately since it's manipulated with GPIO calls

#ifdef FT800P
    triggerPin = 2;				// Used for oscilloscope/logic analyzer trigger
    ft800irqPin = 3;				// Interrupt from FT800 to Arduino - not used here
    ft800pwrPin = 4;				// PD_N from Arduino to FT800 - effectively FT800 reset
    ft800csPin  = 9;				// SPI chip select - defined separately since it's manipulated with GPIO calls
#endif

// LCD display parameters
#ifdef LCD_QVGA					// QVGA display parameters
    lcdWidth   = 320;				// Active width of LCD display
    lcdHeight  = 240;				// Active height of LCD display
    lcdHcycle  = 408;				// Total number of clocks per line
    lcdHoffset = 70;				// Start of active line
    lcdHsync0  = 0;				// Start of horizontal sync pulse
    lcdHsync1  = 10;				// End of horizontal sync pulse
    lcdVcycle  = 263;				// Total number of lines per screen
    lcdVoffset = 13;				// Start of active screen
    lcdVsync0  = 0;				// Start of vertical sync pulse
    lcdVsync1  = 2;				// End of vertical sync pulse
    lcdPclk    = 8;				// Pixel Clock
    lcdSwizzle = 2;				// Define RGB output pins
    lcdPclkpol = 0;				// Define active edge of PCLK
#endif

#ifdef LCD_WQVGA				// WQVGA display parameters
    lcdWidth   = 480;				// Active width of LCD display
    lcdHeight  = 272;				// Active height of LCD display
    lcdHcycle  = 548;				// Total number of clocks per line
    lcdHoffset = 43;				// Start of active line
    lcdHsync0  = 0;				// Start of horizontal sync pulse
    lcdHsync1  = 41;				// End of horizontal sync pulse
    lcdVcycle  = 292;				// Total number of lines per screen
    lcdVoffset = 12;				// Start of active screen
    lcdVsync0  = 0;				// Start of vertical sync pulse
    lcdVsync1  = 10;				// End of vertical sync pulse
    lcdPclk    = 5;				// Pixel Clock
    lcdSwizzle = 0;				// Define RGB output pins
    lcdPclkpol = 1;				// Define active edge of PCLK
#endif

    SPI.begin(ft800csPin);					// Initialize SPI
    SPI.setBitOrder(MSBFIRST);			// Send data most significant bit first
    SPI.setDataMode(SPI_MODE0);			// Clock idle zero, clock data into FT800 on rising edge
    SPI.setClockDivider(SPI_CLOCK_DIV4);		// Set rate at 4MHz (16MHz OSC / 4)

    pinMode(triggerPin, OUTPUT);			// Arduino pin used for oscilloscope triggering
    pinMode(ft800irqPin, INPUT_PULLUP);		// FT800 interrupt output (not used in this example)
    pinMode(ft800pwrPin, OUTPUT);			// FT800 Power Down (reset) input
    pinMode(ft800csPin, OUTPUT);			// FT800 SPI bus CS# input

    digitalWrite(triggerPin, LOW);		// Initialize the oscilloscope trigger
    digitalWrite(ft800csPin, HIGH);		// Set CS# high to start - SPI inactive
    digitalWrite(ft800pwrPin, HIGH);		// Set PD# high to start
    delay(20);					// Wait a few MS before waking the FT800

    //***************************************
// Wake-up FT800

    digitalWrite(ft800pwrPin, LOW);		// 1) lower PD#
    delay(20);					// 2) hold for 20ms
    digitalWrite(ft800pwrPin, HIGH);		// 3) raise PD#
    delay(20);					// 4) wait for another 20ms before sending any commands

    ft800cmdWrite(FT800_ACTIVE);			// Start FT800
    delay(5);					// Give some time to process
    ft800cmdWrite(FT800_CLKEXT);			// Set FT800 for external clock
    delay(5);					// Give some time to process
    ft800cmdWrite(FT800_CLK48M);			// Set FT800 for 48MHz PLL
    delay(5);					// Give some time to process
      					// Now FT800 can accept commands at up to 30MHz clock on SPI bus
      					//   This application leaves the SPI bus at 4MHz

    if (ft800memRead8(REG_ID) != 0x7C)		// Read ID register - is it 0x7C?
    {
        Serial.write("System Halted!\r\n");		// Send an error message on the UART
        while(1);					// If we don't get 0x7C, the ineface isn't working - halt with infinite loop
    }

    ft800memWrite8(REG_PCLK, ZERO);		// Set PCLK to zero - don't clock the LCD until later
    ft800memWrite8(REG_PWM_DUTY, ZERO);		// Turn off backlight

// End of Wake-up FT800
//***************************************

//***************************************
// Initialize Display
    ft800memWrite16(REG_HSIZE,   lcdWidth);	// active display width
    ft800memWrite16(REG_HCYCLE,  lcdHcycle);	// total number of clocks per line, incl front/back porch
    ft800memWrite16(REG_HOFFSET, lcdHoffset);	// start of active line
    ft800memWrite16(REG_HSYNC0,  lcdHsync0);	// start of horizontal sync pulse
    ft800memWrite16(REG_HSYNC1,  lcdHsync1);	// end of horizontal sync pulse
    ft800memWrite16(REG_VSIZE,   lcdHeight);	// active display height
    ft800memWrite16(REG_VCYCLE,  lcdVcycle);	// total number of lines per screen, incl pre/post
    ft800memWrite16(REG_VOFFSET, lcdVoffset);	// start of active screen
    ft800memWrite16(REG_VSYNC0,  lcdVsync0);	// start of vertical sync pulse
    ft800memWrite16(REG_VSYNC1,  lcdVsync1);	// end of vertical sync pulse
    ft800memWrite8(REG_SWIZZLE,  lcdSwizzle);	// FT800 output to LCD - pin order
    ft800memWrite8(REG_PCLK_POL, lcdPclkpol);	// LCD data is clocked in on this PCLK edge
    ft800memWrite32(REG_ROTATE, 0);
						// Don't set PCLK yet - wait for just after the first display list
// End of Initialize Display
//***************************************

//***************************************
// Configure Touch and Audio - not used in this example, so disable both
  ft800memWrite8(REG_TOUCH_MODE, ZERO);		// Disable touch
  ft800memWrite16(REG_TOUCH_RZTHRESH, ZERO);	// Eliminate any false touches

  ft800memWrite8(REG_VOL_PB, ZERO);		// turn recorded audio volume down
  ft800memWrite8(REG_VOL_SOUND, ZERO);		// turn synthesizer volume down
  ft800memWrite16(REG_SOUND, 0x6000);		// set synthesizer to mute

// End of Configure Touch and Audio
//***************************************

//***************************************
// Write Initial Display List & Enable Display

  ramDisplayList = RAM_DL;			// start of Display List
  ft800memWrite32(ramDisplayList, DL_CLEAR_RGB); // Clear Color RGB   00000010 RRRRRRRR GGGGGGGG BBBBBBBB  (R/G/B = Colour values) default zero / black
  ramDisplayList += 4;				// point to next location
  ft800memWrite32(ramDisplayList, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));	// Clear 00100110 -------- -------- -----CST  (C/S/T define which parameters to clear)
  ramDisplayList += 4;				// point to next location
  ft800memWrite32(ramDisplayList, DL_DISPLAY);	// DISPLAY command 00000000 00000000 00000000 00000000 (end of display list)

  ft800memWrite32(REG_DLSWAP, DLSWAP_FRAME);	// 00000000 00000000 00000000 000000SS  (SS bits define when render occurs)
						// Nothing is being displayed yet... the pixel clock is still 0x00
  ramDisplayList = RAM_DL;			// Reset Display List pointer for next list

  ft800Gpio = ft800memRead8(REG_GPIO);		// Read the FT800 GPIO register for a read/modify/write operation
  ft800Gpio = ft800Gpio | 0x80;			// set bit 7 of FT800 GPIO register (DISP) - others are inputs
  ft800memWrite8(REG_GPIO, ft800Gpio);		// Enable the DISP signal to the LCD panel
  ft800memWrite8(REG_PCLK, lcdPclk);		// Now start clocking data to the LCD panel
  for(int duty = 0; duty <= 128; duty++)
  {
    ft800memWrite8(REG_PWM_DUTY, duty);		// Turn on backlight - ramp up slowly to full brighness
    delay(10);
  }
  myTimer.begin(increment, 1000*30/91*2, hmSec);
// End of Write Initial Display List & Enable Display
//***************************************

}

void increment() {
    if(level>120)
      level=29;
      else
      level++;
}

void loop() {
     //sprintf(pubString,"{\"t\":%d,\"g\":%d}",24,5);
    //Particle.publish("DATA",pubString);
  // Wait for graphics processor to complete executing the current command list
  // This happens when REG_CMD_READ matches REG_CMD_WRITE, indicating that all commands
  // have been executed.  The next commands get executed when REG_CMD_WRITE is updated again...
  // then REG_CMD_READ again catches up to REG_CMD_WRITE
  // This is a 4Kbyte ring buffer, so keep pointers within the 4K roll-over
  do
  {
    cmdBufferRd = ft800memRead16(REG_CMD_READ);	// Read the graphics processor read pointer
    cmdBufferWr = ft800memRead16(REG_CMD_WRITE); // Read the graphics processor write pointer
  }while (cmdBufferWr != cmdBufferRd);		// Wait until the two registers match

  cmdOffset = cmdBufferWr;			// The new starting point the first location after the last command

  /*if (millis() >= DS18B20nextSampleTime){
    getTemp();
    //color_target=celsius>shower_temp?0xF51Df1:0x0091D3;
    color_target = celsius<plant_temp_min?0x99ccff:(celsius>plant_temp_max?0xff3300:0x1aff1a);
    COLOR=color_target;
  }*/
  COLOR = 0x99ccff;
 /* if(millis() >= nextUpdateTime){
      nextUpdateTime = millis() + (shower_time*1000*.5)/((120-29+5));
      if(level>120)
      level=29;
      else
      level++;
      Serial.println(level);
  }*/

  /* Drawing begins */

  ft800memWrite32(RAM_CMD + cmdOffset, (CMD_DLSTART));
  cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
  cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  //ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR_RGB | BLACK));
  //cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  //ft800memWrite32(RAM_CMD + cmdOffset, (DL_SCISSOR_SIZE | level<<12 | 480));
  //cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  //ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
  //cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR_RGB | COLOR));
  cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800memWrite32(RAM_CMD + cmdOffset, (DL_CLEAR | CLR_COL | CLR_STN | CLR_TAG));
  cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800memWrite32(RAM_CMD + cmdOffset, (CMD_NUMBER));
  cmdOffset = incCMDOffset(cmdOffset, 4);

  ft800memWrite32(RAM_CMD + cmdOffset, (3*level));
  cmdOffset = incCMDOffset(cmdOffset, 2);

  ft800memWrite32(RAM_CMD + cmdOffset, (145));
  cmdOffset = incCMDOffset(cmdOffset, 2);

  ft800memWrite32(RAM_CMD + cmdOffset, (31));
  cmdOffset = incCMDOffset(cmdOffset, 2);	// Update the command pointer

  ft800memWrite32(RAM_CMD + cmdOffset, (OPT_CENTERY));
  cmdOffset = incCMDOffset(cmdOffset, 2);	// Update the command pointer

  ft800memWrite32(RAM_CMD + cmdOffset, ((int)(celsius)));
  cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer


  ft800memWrite32(RAM_CMD + cmdOffset, (DL_DISPLAY));
  cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer


  ft800memWrite32(RAM_CMD + cmdOffset, (CMD_SWAP));
  cmdOffset = incCMDOffset(cmdOffset, 4);	// Update the command pointer

  ft800memWrite16(REG_CMD_WRITE, (cmdOffset));	// Update the ring buffer pointer so the graphics processor starts executing
				// Wait a half-second to observe the changing color
}
