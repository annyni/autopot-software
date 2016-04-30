#include "application.h"
#include "FT800.h"
#include "SparkIntervalTimer.h"
#include "DS18B20.h"
#include "AdaFruit_Sensor.h"
#include "TSL2591.h"
#include "CHIRP.h"
#include "I2CSoilMoistureSensor.h"

// Delcarations
// Also see FT800_hw.h and FT800_sw.h
//
#define TEST_TEMP_VALUE 20

// sensor declarations
//#define TSL2591_CONN

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

#ifdef TSL2591_CONN
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);
unsigned int TSL2591nextSampleTime;
unsigned int TSL2591_SAMPLE_INTERVAL = 600;
#endif

CHIRP chirp = CHIRP();
unsigned int chirpMoisture;
unsigned int chirpTemp;
unsigned int chirpLight;

//I2CSoilMoistureSensor i2cChirp = I2CSoilMoistureSensor();

double celsius;
double fahrenheit;
uint16_t  tslIR;
uint16_t  tslFull;
uint32_t  tslLux;
char pubString[15];
unsigned int level=29;
unsigned int COLOR=0x0091D3;
unsigned int color_target=0x0091D3;
unsigned int plant_temp_max= 85;
unsigned int plant_temp_min= 70;
// Global Variables

// Arduino pins - others defined by Serial and SPI libraries
/*unsigned int triggerPin;			// Used for oscilloscope/logic analyzer trigger
unsigned int ft800irqPin;			// Interrupt from FT800 to Arduino - not used here
unsigned int ft800pwrPin;			// PD_N from Arduino to FT800 - effectively FT800 reset
unsigned int ft800csPin;			// SPI chip select - defined separately since it's manipulated with GPIO calls
*/
// LCD display parameters
/*unsigned int lcdWidth;				// Active width of LCD display
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
*/
unsigned int cmdBufferRd = 0x0000;		// Used to navigate command ring buffer
unsigned int cmdBufferWr = 0x0000;		// Used to navigate command ring buffer
unsigned int cmdOffset = 0x0000;		// Used to navigate command rung buffer
unsigned int point_size = 20;		// Define a default dot size
unsigned long point_x = (96 * 16);		// Define a default point x-location (1/16 anti-aliased)
unsigned long point_y = (136 * 16);		// Define a default point y-location (1/16 anti-aliased)
unsigned long color;				// Variable for chanign colors
//unsigned char ft800Gpio;			// Used for FT800 GPIO register

// Particle variables
double tempC;
unsigned int tempMin;
unsigned int tempMax;
uint16_t lightFull1 = 0;
//unsigned int lightFull2 = 0;
unsigned int moistReading = 0;
//flow rate?

int recvTempMin(String m);
int recvTempMax(String m);

/*void increment() {
    if(level>120)
      level=29;
      else
      level++;
}*/
void setup() {
    tempC = 0;
    tempMin = plant_temp_min;
    tempMax = plant_temp_max;
    //Particle.variable("messageSent", &messageSent, STRING);
    Serial.begin(9600);
    Serial.println("Debugging====");
    //i2cChirp.begin();
    /* Set up for particle-webapp interaction */
    Particle.function("sendTempMin", recvTempMin);
    Particle.function("sendTempMax", recvTempMax);
    Particle.variable("getTemper", &tempC, DOUBLE);
    pinMode(P1S4, OUTPUT);
    digitalWrite(P1S4,LOW);
    Time.zone(-4);
    DS18B20nextSampleTime=millis();
    #ifdef TSL2591_CONN
    TSL2591nextSampleTime=millis();
    #endif
    nextUpdateTime=millis();
    Particle.syncTime();
    pinMode(D2, INPUT);
    //ft800.triggerPin = 2;				// Used for oscilloscope/logic analyzer trigger

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

    digitalWrite(ft800.ft800pwrPin, LOW);		// 1) lower PD#
    delay(20);					// 2) hold for 20ms
    digitalWrite(ft800.ft800pwrPin, HIGH);		// 3) raise PD#
    delay(20);					// 4) wait for another 20ms before sending any commands
    ft800.init(ft800);

    //Particle.variable("getMoisture", &moistReading, INT);
    //Particle.variable("getLight1", &lightFull1, INT);
    //Particle.variable("getLight2", &lightFull2, INT);

    //unsigned int tempMin = 0;
    //unsigned int tempMax = 0;
    //uint16_t lightFull1 = 0;
    //unsigned int lightFull2 = 0;
    //unsigned int moistReading = 0;
}


int recvTempMin(String m){
    Serial.println("initial tempmin msg:" + m);
    tempMin = (unsigned int)atoi(m);
    Serial.println("tempMin received:");
    Serial.println(tempMin);
    return 0;
}

int recvTempMax(String m) {
    Serial.println("initial tempmax msg:" + m);
    tempMax = (unsigned int)atoi(m);
    Serial.println("tempMax received:");
    Serial.println(tempMax);
    return 0;
}

void getTemp(){
    if(!ds18b20.search()){
      ds18b20.resetsearch();
      int check = ds18b20.getTemperature();
      celsius = ds18b20.getTemperature();
      fahrenheit = ds18b20.convertToFahrenheit(celsius);
      tempC = fahrenheit;
      DS18B20nextSampleTime = millis() + DS18B20_SAMPLE_INTERVAL;
      //Serial.println(fahrenheit);
    }
}


// helper function for tsl2591
#ifdef TSL2591_CONN
void getLight() {
  if(tsl2591.begin()) {
    Serial.println("inside");
    tsl2591.getReading();
    tslLux= tsl2591._lux;
    tslIR = tsl2591._ir;
    tslFull = tsl2591._full;
    lightFull1 = (uint16_t)tslFull;
    TSL2591nextSampleTime = millis() + TSL2591_SAMPLE_INTERVAL;
  }
}
#endif

/*void getMoisture() {
  chirp.setup();
    chirp.loop();
    chirpLight = chirp._light;
    chirpTemp = chirp._temp;
    chirpMoisture = chirp._cap;
*/
/*    Serial.print("Soil Moisture Capacitance: ");
  Serial.print(i2cChirp.getCapacitance()); //read capacitance register
  Serial.print(", Temperature: ");
  Serial.print(i2cChirp.getTemperature()/(float)10); //temperature register
  Serial.print(", Light: ");
  Serial.println(i2cChirp.getLight(true)); //request light measurement, wait and read light register*/
//}

void loop() {
  do
  {
    cmdBufferRd = ft800.ft800memRead16(REG_CMD_READ);	// Read the graphics processor read pointer
    cmdBufferWr = ft800.ft800memRead16(REG_CMD_WRITE); // Read the graphics processor write pointer
  }while (cmdBufferWr != cmdBufferRd);		// Wait until the two registers match

  cmdOffset = cmdBufferWr;			// The new starting point the first location after the last command

  if (millis() >= DS18B20nextSampleTime){
    getTemp();
    color_target = fahrenheit<tempMin?0x99ccff:(fahrenheit>tempMax?0xff3300:0x1aff1a);
    COLOR=color_target;
  }

  //getMoisture();

  //Serial.printf("moisture: %d, temp: %d, light: %d \n", chirpMoisture, chirpTemp, chirpLight);
  // update tsl2591 light
  #ifdef TSL2591_CONN
  if (millis() >= TSL2591nextSampleTime){
    getLight();
    Serial.printf("Lux: %d, Full: %d, IR: %d \n", tslLux, tslFull, tslIR);
  }
  #endif

  /* Drawing begins */
  ft800.draw(ft800, cmdOffset, COLOR, fahrenheit, plant_temp_min, plant_temp_max);

}
