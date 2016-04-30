#include "CHIRP.h"
#include "application.h"

CHIRP::CHIRP() {

}

void CHIRP::setup() {
  Wire.begin();
  //writeI2CRegister8bit(0x20, 6); //reset
  //Serial.begin(9600);
}

void CHIRP::writeI2CRegister8bit(int addr, int value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

unsigned int CHIRP::readI2CRegister16bit(int addr, int reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(1000);
  Wire.requestFrom(addr, 2);
  unsigned int t = Wire.read() << 8;
  t = t | Wire.read();
  return t;
}

void CHIRP::loop() {
  Serial.print(readI2CRegister16bit(0x20, 0)); //read capacitance register
  Serial.print(", ");
  Serial.print(readI2CRegister16bit(0x20, 5)); //temperature register
   delay(9000);
  Serial.print(", ");
  writeI2CRegister8bit(0x20, 3); //request light measurement
   delay(9000);
  Serial.println(readI2CRegister16bit(0x20, 4)); //read light register

  _cap = readI2CRegister16bit(0x20, 0);
  //Serial.print(_cap); //read capacitance register
  _temp = readI2CRegister16bit(0x20, 5);
  //Serial.print(_temp); //temperature register
  writeI2CRegister8bit(0x20, 3); //request light measurement
  _light = readI2CRegister16bit(0x20, 4);
  //Serial.println(_temp); //read light register
  delay(500);
}

/*
CHIRP::CHIRP(int32_t _sensorID)
{
    _init = false;
}

bool CHIRP::init(void)
{
    Wire.begin();

    _cap = 0;
    _temp = 0;
    _light = 0;
    uint8_t write={CHIRP_CMD_RESET};
    if(write8(CHIRP_ADDR, write))
        return false;

    _init = true;
    return true;
}

void CHIRP::getCap(void)
{
    if(!_init)
        if(!init())
            return;
    uint8_t write={CHIRP_REG_CAP};
    uint8_t read[2];
    write8(CHIRP_ADDR, write);
    read = read8(CHIRP_ADDR);
    _cap = (read[0]<<8)|(read[1]);
}

void CHIRP::getTemp(void)
{
    if(!_init)
        if(!init())
            return;

    uint8_t write={CHIRP_REG_TEMP};
    uint8_t read[2];
    write8(CHIRP_ADDR, write);
    read = read8(CHIRP_ADDR);
    _temp = (read[0]<<8)|(read[1]);
}

void CHIRP::getLight(void)
{
    if(!_init)
        if(!init())
            return;
    uint8_t write={CHIRP_CMD_LIGHT};
    write8(CHIRP_ADDR);
    write[0]=CHIRP_REG_LIGHT;
    uint8_t read[2];
    write8(CHIRP_ADDR, write);
    read = read8(CHIRP_ADDR);
    _light = (read[0]<<8)|(read[1]);
}


void CHIRP::write8 (uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(CHIRP_ADDR);
#if ARDUINO >= 100 || SPARK || PARTICLE
  Wire.write(reg);
  Wire.write(value);
#else
  Wire.send(reg);
  Wire.send(value);
#endif
  Wire.endTransmission();
}


uint8_t CHIRP::read8(uint8_t reg)
{
  uint8_t x;

  Wire.beginTransmission(CHIRP_ADDR);
#if ARDUINO >= 100 || SPARK || PARTICLE
  Wire.write(reg);
#else
  Wire.send(reg);
#endif
  Wire.endTransmission();

  Wire.requestFrom(CHIRP_ADDR, 1);
#if ARDUINO >= 100 || SPARK || PARTICLE
  x = Wire.read();
#else
  x = Wire.receive();
#endif
  // while (! Wire.available());
  // return Wire.read();
  return x;
}
*/
