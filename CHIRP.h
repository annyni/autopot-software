#ifndef CHIRP_H
#define CHIRP_H

#include "application.h"

#define CHIRP_ADDR      0x20
#define CHIRP_REG_CAP   0x00
#define CHIRP_REG_TEMP  0x05
#define CHIRP_REG_LIGHT 0x04
#define CHIRP_CMD_LIGHT 0x03
#define CHIRP_CMD_RESET 0x06

class CHIRP
{
public:
  CHIRP();
  void setup();
  void writeI2CRegister8bit(int addr, int value);
  unsigned int readI2CRegister16bit(int addr, int reg);
  void loop();

    uint8_t   read8   ( uint8_t reg );
    void   write8   ( uint8_t reg, uint8_t value );

    bool init(void);
    unsigned int _cap;
    unsigned int _temp;
    unsigned int _light;


protected:
    //I2C _i2c;
    int32_t _sensorID;
    bool _init;
};

#endif
