#include "application.h"
#include "FT800.h"
#include "SparkIntervalTimer.h"
#include "DS18B20.h"
#include "AdaFruit_Sensor.h"
#include "TSL2591.h"

// Delcarations
// Also see FT800_hw.h and FT800_sw.h
//
#define TEST_TEMP_VALUE 20
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

// Sensors
DS18B20 ds18b20 = DS18B20(D2);
unsigned int DS18B20nextSampleTime;
unsigned int DS18B20_SAMPLE_INTERVAL = 600;
unsigned int nextUpdateTime;
unsigned int UPDATE_INTERVAL = 300;
FT800 ft800 = FT800();

Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);
unsigned int TSL2591nextSampleTime;
unsigned int TSL2591_SAMPLE_INTERVAL = 600;

double celsius;
double fahrenheit;
uint16_t  tslIR;
uint16_t  tslFull;
uint32_t  tslLux;
char pubString[15];
unsigned int level=29;
unsigned int COLOR=0x0091D3;
unsigned int color_target=0x0091D3;
unsigned int plant_temp_max= 33;
unsigned int plant_temp_min= 23;
unsigned int shower_time=30;
unsigned int shower_temp=38;
// Global Variables

// Arduino pins - others defined by Serial and SPI libraries
/*unsigned int triggerPin;			// Used for oscilloscope/logic analyzer trigger
unsigned int ft800irqPin;			// Interrupt from FT800 to Arduino - not used here
unsigned int ft800pwrPin;			// PD_N from Arduino to FT800 - effectively FT800 reset
unsigned int ft800csPin;			// SPI chip select - defined separately since it's manipulated with GPIO calls
*/
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


void setup() {
    pinMode(P1S4, OUTPUT);
    digitalWrite(P1S4,LOW);
    Time.zone(-4);
    DS18B20nextSampleTime=millis();
    TSL2591nextSampleTime=millis();
    nextUpdateTime=millis();
    Particle.syncTime();
    pinMode(D2, INPUT);
    // need change
    //ft800.triggerPin = 2;				// Used for oscilloscope/logic analyzer trigger
    ft800.ft800irqPin = A2;				// Interrupt from FT800 to Arduino - not used here
    ft800.ft800pwrPin = A0;				// PD_N from Arduino to FT800 - effectively FT800 reset
    ft800.ft800csPin  = DAC;				// SPI chip select - defined separately since it's manipulated with GPIO calls

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

    SPI.begin(ft800.ft800csPin);					// Initialize SPI
    SPI.setBitOrder(MSBFIRST);			// Send data most significant bit first
    SPI.setDataMode(SPI_MODE0);			// Clock idle zero, clock data into FT800 on rising edge
    SPI.setClockDivider(SPI_CLOCK_DIV4);		// Set rate at 4MHz (16MHz OSC / 4)

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
  //myTimer.begin(increment, 1000*30/91*2, hmSec);
// End of Write Initial Display List & Enable Display
//***************************************
}

void increment() {
    if(level>120)
      level=29;
      else
      level++;
}

void getTemp(){
    if(!ds18b20.search()){
      ds18b20.resetsearch();
      celsius = ds18b20.getTemperature();
      fahrenheit = ds18b20.convertToFahrenheit(celsius);
      DS18B20nextSampleTime = millis() + DS18B20_SAMPLE_INTERVAL;
      //Serial.println(fahrenheit);
    }
}

void getLight() {
  if(!tsl2591.begin()) {
    tsl2591.getReading();
    tslLux= tsl2591.lux;
    tslIR = tsl2591._ir;
    tslFull = tsl2591._full;
    TSL2591nextSampleTime = millis() + TSL2591_SAMPLE_INTERVAL;
  }
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
    cmdBufferRd = ft800.ft800memRead16(REG_CMD_READ);	// Read the graphics processor read pointer
    cmdBufferWr = ft800.ft800memRead16(REG_CMD_WRITE); // Read the graphics processor write pointer
  }while (cmdBufferWr != cmdBufferRd);		// Wait until the two registers match

  cmdOffset = cmdBufferWr;			// The new starting point the first location after the last command

  if (millis() >= DS18B20nextSampleTime){
    getTemp();
    color_target = celsius<plant_temp_min?0x99ccff:(celsius>plant_temp_max?0xff3300:0x1aff1a);
    COLOR=color_target;
  }

  if (millis() >= TSL2591nextSampleTime){
    getLight();
    Serial.println("Lux: " + tslLux +", Full: " + tslFull + ",IR: " + tslIR);
  }
  //celsius = TEST_TEMP_VALUE;
  //color_target = celsius<plant_temp_min?0x99ccff:(celsius>plant_temp_max?0xff3300:0x1aff1a);
  //COLOR=color_target;
 /* if(millis() >= nextUpdateTime){
      nextUpdateTime = millis() + (shower_time*1000*.5)/((120-29+5));
      if(level>120)
      level=29;
      else
      level++;
      Serial.println(level);
  }*/

  /* Drawing begins */
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
          cmdOffset = ft800.displayText(ft800, RAM_CMD, "i'm cold! move me! :S", 10, 145, cmdOffset);
        } else if (celsius > plant_temp_max) {
          cmdOffset = ft800.displayText(ft800, RAM_CMD, "it's too hot! help! :(", 10, 145, cmdOffset);
        } else {
          cmdOffset = ft800.displayText(ft800, RAM_CMD, "i'm happy :)", 10, 145, cmdOffset);
        }

        ft800.ft800memWrite32(RAM_CMD + cmdOffset, (DL_DISPLAY));
        cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer


        ft800.ft800memWrite32(RAM_CMD + cmdOffset, (CMD_SWAP));
        cmdOffset = ft800.incCMDOffset(cmdOffset, 4);	// Update the command pointer

        ft800.ft800memWrite16(REG_CMD_WRITE, (cmdOffset));
}
