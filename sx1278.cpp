#include "sx1278.h"

const int NSS = 18;
static SPISettings spiset = SPISettings(10000000L, SPI_MSBFIRST, SPI_MODE0);

uint8_t readRegister(uint8_t reg) {
  digitalWrite(NSS,LOW);
  SPI.beginTransaction(spiset);
  SPI.transfer(reg);
  uint8_t res=SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(NSS,HIGH);
  return res;
}

void writeRegister(uint8_t reg,uint8_t val) {
  digitalWrite(NSS,LOW);
  SPI.beginTransaction(spiset);
  SPI.transfer(0x80|reg);
  SPI.transfer(val);
  SPI.endTransaction();
  digitalWrite(NSS,HIGH);
}
