#include "CHIRP.h"
#include "application.h"

CHIRP::CHIRP() {

}

void CHIRP::setup() {
  Wire.begin();
  //writeI2CRegister8bit(0x20, 6);
  //Serial.begin(9600);
  //reset pin 48
  pinMode(P1S5, OUTPUT);
  digitalWrite(P1S5, LOW);  //reset chirp
  //delay(1); maybe allow some time for chirp to reset
  digitalWrite(P1S5, HIGH); //Go out from reset
  writeI2CRegister8bit(0x20, 3); //send something on the I2C bus
  delay(1000); //allow chirp to boot
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
  delay(1100);
  Wire.requestFrom(addr, 2);
  unsigned int t = Wire.read() << 8;
  t = t | Wire.read();
  return t;
}

void CHIRP::loop() {
  Serial.print("CAP:");
  Serial.print(readI2CRegister16bit(0x20, 0)); //read capacitance register
  Serial.print(", ");
  Serial.print("TEMP:");
  Serial.print(readI2CRegister16bit(0x20, 5)); //temperature register

  //_cap = readI2CRegister16bit(0x20, 0);
  //Serial.print(_cap); //read capacitance register
  //_temp = readI2CRegister16bit(0x20, 5);
  //Serial.print(_temp); //temperature register
  //writeI2CRegister8bit(0x20, 3); //request light measurement
  //_light = readI2CRegister16bit(0x20, 4);
  //Serial.println(_temp); //read light register
  //delay(500);
}
