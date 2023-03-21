#include "mv300sensor.h"
#include <math.h>

uint8_t crcPolynomialLookupTable[256] ={0x00,0x07,0x0e,0x09,0x1c,0x1b,0x12,0x15,0x38,0x3f,0x36,0x31,0x24,0x23,0x2a,0x2d,
                                        0x70,0x77,0x7e,0x79,0x6c,0x6b,0x62,0x65,0x48,0x4f,0x46,0x41,0x54,0x53,0x5a,0x5d,
                                        0xe0,0xe7,0xee,0xe9,0xfc,0xfb,0xf2,0xf5,0xd8,0xdf,0xd6,0xd1,0xc4,0xc3,0xca,0xcd,
                                        0x90,0x97,0x9e,0x99,0x8c,0x8b,0x82,0x85,0xa8,0xaf,0xa6,0xa1,0xb4,0xb3,0xba,0xbd,
                                        0xc7,0xc0,0xc9,0xce,0xdb,0xdc,0xd5,0xd2,0xff,0xf8,0xf1,0xf6,0xe3,0xe4,0xed,0xea,
                                        0xb7,0xb0,0xb9,0xbe,0xab,0xac,0xa5,0xa2,0x8f,0x88,0x81,0x86,0x93,0x94,0x9d,0x9a,
                                        0x27,0x20,0x29,0x2e,0x3b,0x3c,0x35,0x32,0x1f,0x18,0x11,0x16,0x03,0x04,0x0d,0x0a,
                                        0x57,0x50,0x59,0x5e,0x4b,0x4c,0x45,0x42,0x6f,0x68,0x61,0x66,0x73,0x74,0x7d,0x7a,
                                        0x89,0x8e,0x87,0x80,0x95,0x92,0x9b,0x9c,0xb1,0xb6,0xbf,0xb8,0xad,0xaa,0xa3,0xa4,
                                        0xf9,0xfe,0xf7,0xf0,0xe5,0xe2,0xeb,0xec,0xc1,0xc6,0xcf,0xc8,0xdd,0xda,0xd3,0xd4,
                                        0x69,0x6e,0x67,0x60,0x75,0x72,0x7b,0x7c,0x51,0x56,0x5f,0x58,0x4d,0x4a,0x43,0x44,
                                        0x19,0x1e,0x17,0x10,0x05,0x02,0x0b,0x0c,0x21,0x26,0x2f,0x28,0x3d,0x3a,0x33,0x34,
                                        0x4e,0x49,0x40,0x47,0x52,0x55,0x5c,0x5b,0x76,0x71,0x78,0x7f,0x6a,0x6d,0x64,0x63,
                                        0x3e,0x39,0x30,0x37,0x22,0x25,0x2c,0x2b,0x06,0x01,0x08,0x0f,0x1a,0x1d,0x14,0x13,
                                        0xae,0xa9,0xa0,0xa7,0xb2,0xb5,0xbc,0xbb,0x96,0x91,0x98,0x9f,0x8a,0x8d,0x84,0x83,
                                        0xde,0xd9,0xd0,0xd7,0xc2,0xc5,0xcc,0xcb,0xe6,0xe1,0xe8,0xef,0xfa,0xfd,0xf4,0xf3};

const double convertRadiansToDegrees=180.0/M_PI;

int16_t twosComplement(uint16_t value, uint8_t numberOfBits) {
  if ((value & (1 << (numberOfBits - 1))) != 0) {
    value = value - (1 << numberOfBits);
  }
  return value;
}

double convertMagneticFieldFromLsbToMilliTesla(int16_t magneticFieldInLsb, uint8_t brange) {
  if (brange == 0) {
    return magneticFieldInLsb/7.0;
  }
  else {
    return magneticFieldInLsb/14.0;
  }
}

double convertTemperatureFromLsbToDegreeCelsius(int16_t temperatureInLsb) {
  return (temperatureInLsb/5.0)-41.0;
}

void computeNormThetaPhi(double magFieldBx, double magFieldBy, double magFieldBz, double *norm, double *theta, double *phi) {
  *norm = sqrt(pow(magFieldBx, 2)+pow(magFieldBy, 2)+pow(magFieldBz, 2));
  *theta = acos(magFieldBz/(*norm))*convertRadiansToDegrees;
  *phi = fmod((atan2(magFieldBy, magFieldBx)*convertRadiansToDegrees)+360.0, 360.0);
}

MV300SensorI2c::MV300SensorI2c() {
  i2cAddress=0x14;
}

void MV300SensorI2c::begin() {
  Wire.begin();
  setClockFrequency(400000);
  //Wire.setClock(100000);//Standard-mode (Sm)
  //Wire.setClock(400000);//Fast-mode (Fm)
  //Wire.setClock(1000000);//Fast-mode Plus (Fm) WARNING: To be tested
}

void MV300SensorI2c::end() {
  Wire.end();
}

void MV300SensorI2c::setDeviceAddress(uint8_t deviceAddress) {
  i2cAddress=deviceAddress;
}

uint8_t MV300SensorI2c::getDeviceAddress() {
  return i2cAddress;
}

void MV300SensorI2c::setClockFrequency(uint32_t clockFrequency) {
  i2cClockFrequency=clockFrequency;
  Wire.setClock(i2cClockFrequency);
}

uint32_t MV300SensorI2c::getClockFrequency() {
  return i2cClockFrequency;
}

void MV300SensorI2c::readMagneticComponents(uint16_t *magFieldBx, uint16_t *magFieldBy, uint16_t *magFieldBz, uint16_t *temperature, uint8_t *frameCounter, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(((triggerMode<<6)&0xC0) | (0&0x3F)));
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 7);
  while(Wire.available()) {
    readRegistersArray[byteCount] = Wire.read();
    byteCount++;
  }
  *magFieldBx=(readRegistersArray[0]<<4) | ((readRegistersArray[4]&0xF0)>>4);
  *magFieldBy=(readRegistersArray[1]<<4) | (readRegistersArray[4]&0x0F);
  *magFieldBz=(readRegistersArray[2]<<4) | (readRegistersArray[5]&0x0F);
  *temperature=(readRegistersArray[3]<<4) | ((readRegistersArray[5]&0xC0)>>4) | ((readRegistersArray[6]&0x30)>>4);
  *frameCounter=readRegistersArray[6]&0x03;
}

uint8_t MV300SensorI2c::readRegisterQuickReadMode(uint8_t numberOfRegisterToRead, uint8_t *readRegisters) {
  uint8_t byteCount = 0;
  Wire.requestFrom(i2cAddress, numberOfRegisterToRead);
  while(Wire.available()) {
    readRegisters[byteCount] = Wire.read();
    byteCount++;
  }
  return byteCount;
}

uint8_t MV300SensorI2c::readRegisterBurstMode(uint8_t address, uint8_t numberOfRegisterToRead, uint8_t *readRegisters, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(((triggerMode<<6)&0xC0) | (address&0x3F)));
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, numberOfRegisterToRead);
  while(Wire.available()) {
    readRegisters[byteCount] = Wire.read();
    byteCount++;
  }
  return byteCount;
}

void MV300SensorI2c::writeRegisterBurstMode(uint8_t address, uint8_t numberOfRegisterToWrite, uint8_t *registerValueToWrite, uint8_t triggerMode) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(((triggerMode<<6)&0xC0) | ((address)&0x3F)));
  for ( uint8_t i=0;i<numberOfRegisterToWrite; ++i) {
    Wire.write(byte(registerValueToWrite[i]));
  }
  Wire.endTransmission();
}

uint8_t MV300SensorI2c::readRegister(uint8_t address, uint8_t triggerMode) {
  uint8_t readbackValue;
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(((triggerMode<<6)&0xC0) | (address&0x3F)));
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 1, true);
  readbackValue = Wire.read();
  return readbackValue;
}

void MV300SensorI2c::writeRegister(uint8_t address, uint8_t value, uint8_t triggerMode) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(byte(((triggerMode<<6)&0xC0) | (address&0x3F)));
  Wire.write(byte(value));
  Wire.endTransmission();
}

void MV300SensorI2c::readMagneticComponentsWithCrcCheck(uint16_t *magFieldBx, uint16_t *magFieldBy, uint16_t *magFieldBz, uint16_t *temperature, uint8_t *frameCounter, bool *crcErrorDetected, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  uint8_t crc = 0;
  uint8_t crcReceived;
  uint8_t readbackValue;
  uint8_t data;
  *crcErrorDetected=false;
  Wire.beginTransmission(i2cAddress);
  data = ((triggerMode<<6)&0xC0) | (0&0x3F);
  Wire.write(byte(data));
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 7*2, true);
  while(Wire.available()) {
    crc = 0;
    crc = crc8(crc, (i2cAddress<<1)+0);
    data = ((triggerMode<<6)&0xC0) | ((0+byteCount)&0x3F);
    crc = crc8(crc, data);
    crc = crc8(crc, (i2cAddress<<1)+1);
    readbackValue = Wire.read();
    crc = crc8(crc, readbackValue);
    readRegistersArray[byteCount] = readbackValue;
    crcReceived = Wire.read();
    if (crc != crcReceived) {
      *crcErrorDetected=true;
    }
    byteCount++;
  }
  *magFieldBx=(readRegistersArray[0]<<4) | ((readRegistersArray[4]&0xF0)>>4);
  *magFieldBy=(readRegistersArray[1]<<4) | (readRegistersArray[4]&0x0F);
  *magFieldBz=(readRegistersArray[2]<<4) | (readRegistersArray[5]&0x0F);
  *temperature=(readRegistersArray[3]<<4) | ((readRegistersArray[5]&0xC0)>>4) | ((readRegistersArray[6]&0x30)>>4);
  *frameCounter=readRegistersArray[6]&0x03;
}

uint8_t MV300SensorI2c::readRegisterQuickReadModeWithCrcCheck(uint8_t numberOfRegisterToRead, uint8_t* readRegisters, bool *crcErrorDetected) {
  uint8_t byteCount = 0;
  uint8_t crc = 0;
  uint8_t crcReceived;
  uint8_t readbackValue;
  bool dummy = 0;
  uint8_t first_reg = 0;
  uint8_t is_qrsadden;
  *crcErrorDetected=false;
  is_qrsadden = (readRegisterWithCrcCheck(0x11, &dummy, 0) & 0x08) >> 3;
  if (is_qrsadden) first_reg = (readRegisterWithCrcCheck(0x12, &dummy, 0) & 0x1F);
  Wire.requestFrom(i2cAddress, numberOfRegisterToRead*2);
  while(Wire.available()) {
    crc = 0;
    crc = crc8(crc, (i2cAddress<<1)+1);
    crc = crc8(crc, (first_reg + byteCount)&0x3F);
    readbackValue = Wire.read();
    crc = crc8(crc, readbackValue);
    readRegisters[byteCount] = readbackValue;
    crcReceived = Wire.read();
    if (crc != crcReceived) {
      *crcErrorDetected=true;
    }
    byteCount++;
  }
  return byteCount;
}

uint8_t MV300SensorI2c::readRegisterBurstModeWithCrcCheck(uint8_t address, uint8_t numberOfRegisterToRead, uint8_t* readRegisters, bool *crcErrorDetected, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  uint8_t crc = 0;
  uint8_t crcReceived;
  uint8_t readbackValue;
  uint8_t data;
  *crcErrorDetected=false;
  Wire.beginTransmission(i2cAddress);
  data = ((triggerMode<<6)&0xC0) | (address&0x3F);
  Wire.write(byte(data));
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, numberOfRegisterToRead*2, true);
  while(Wire.available()) {
    crc = 0;
    crc = crc8(crc, (i2cAddress<<1)+0);
    data = ((triggerMode<<6)&0xC0) | ((address+byteCount)&0x3F);
    crc = crc8(crc, data);
    crc = crc8(crc, (i2cAddress<<1)+1);
    readbackValue = Wire.read();
    crc = crc8(crc, readbackValue);
    readRegisters[byteCount] = readbackValue;
    crcReceived = Wire.read();
    if (crc != crcReceived) {
      *crcErrorDetected=true;
    }
    byteCount++;
  }
  return byteCount;
}

uint8_t MV300SensorI2c::readRegisterWithCrcCheck(uint8_t address, bool *crcErrorDetected, uint8_t triggerMode) {
  uint8_t crc = 0;
  uint8_t crcReceived;
  uint8_t readbackValue;
  uint8_t data;
  *crcErrorDetected=false;
  Wire.beginTransmission(i2cAddress);
  crc = crc8(crc, (i2cAddress<<1)+0);
  data=((triggerMode<<6)&0xC0) | (address&0x3F);
  Wire.write(byte(data));
  crc = crc8(crc, data);
  Wire.endTransmission(false);
  Wire.requestFrom(i2cAddress, 2, true);
  crc = crc8(crc, (i2cAddress<<1)+1);
  readbackValue = Wire.read();
  crc = crc8(crc, readbackValue);
  crcReceived = Wire.read();
  if (crc != crcReceived) {
    *crcErrorDetected=true;
  }
  return readbackValue;
}

void MV300SensorI2c::writeRegisterWithCrcCheck(uint8_t address, uint8_t value, uint8_t triggerMode) {
  uint8_t crc = 0;
  uint8_t data;
  Wire.beginTransmission(i2cAddress);
  crc = crc8(crc, (i2cAddress<<1)+0);
  data=((triggerMode<<6)&0xC0) | (address&0x3F);
  Wire.write(byte(data));
  crc = crc8(crc, data);
  Wire.write(byte(value));
  crc = crc8(crc, value);
  Wire.write(byte(crc));
  Wire.endTransmission();
}
 
uint8_t MV300SensorI2c::crc8(uint8_t crc, uint8_t data) {
  return crcPolynomialLookupTable[crc^data];
}

MV300SensorSpi::MV300SensorSpi() {
  
}

void MV300SensorSpi::begin(uint8_t chipSelectPin) {
  setChipSelectPin(chipSelectPin);
  spiMaximumSpeed = 10000000;
  spiDataMode = SPI_MODE0;
  SPI.begin();
  SPI.beginTransaction(SPISettings(spiMaximumSpeed, MSBFIRST, spiDataMode));
}
 
void MV300SensorSpi::begin(int32_t sclkFrequency, uint8_t mode, uint8_t chipSelectPin) {
  setChipSelectPin(chipSelectPin);
  spiMaximumSpeed = sclkFrequency;
  spiDataMode = mode;
  SPI.begin();
  SPI.beginTransaction(SPISettings(spiMaximumSpeed, MSBFIRST, spiDataMode));
}
 
void MV300SensorSpi::end() {
  SPI.end();
}
 
void MV300SensorSpi::readMagneticComponents(uint16_t *magFieldBx, uint16_t *magFieldBy, uint16_t *magFieldBz, uint16_t *temperature, uint8_t *frameCounter, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  uint8_t address=0;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  SPI.transfer(((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (address&0x1F));
  for(int i=0;i<7;++i) {
    address=i+1;
    readRegistersArray[byteCount] = SPI.transfer(((0<<6)&0xC0) | ((1<<5)&0x20) | (address&0x1F));
    byteCount++;
  }
  digitalWrite(spiChipSelectPin, HIGH);
  *magFieldBx=(readRegistersArray[0]<<4) | ((readRegistersArray[4]&0xF0)>>4);
  *magFieldBy=(readRegistersArray[1]<<4) | (readRegistersArray[4]&0x0F);
  *magFieldBz=(readRegistersArray[2]<<4) | (readRegistersArray[5]&0x0F);
  *temperature=(readRegistersArray[3]<<4) | ((readRegistersArray[5]&0xC0)>>4) | ((readRegistersArray[6]&0x30)>>4);
  *frameCounter=readRegistersArray[6]&0x03;
}
 
uint8_t MV300SensorSpi::readRegister(uint8_t address, uint8_t triggerMode) {
  uint8_t readbackValue;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  SPI.transfer(((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (address&0x1F));
  readbackValue = SPI.transfer(((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (0&0x1F));
  digitalWrite(spiChipSelectPin, HIGH);
  return readbackValue;
}
 
void MV300SensorSpi::writeRegister(uint8_t address, uint8_t value, uint8_t triggerMode) {
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  SPI.transfer(((triggerMode<<6)&0xC0) | ((0<<5)&0x20) | (address&0x1F));
  SPI.transfer(value);
  digitalWrite(spiChipSelectPin, HIGH);
}
 
uint8_t MV300SensorSpi::readRegisterBurstMode(uint8_t address, uint8_t numberOfRegisterToRead, uint8_t *readRegisters, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  SPI.transfer(((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (address&0x1F));
  for(int i=0;i<numberOfRegisterToRead;++i) {
    readRegisters[byteCount] = SPI.transfer(((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | ((address+1+i)&0x1F));
    byteCount++;
  }
  digitalWrite(spiChipSelectPin, HIGH);
  return byteCount;
}

void MV300SensorSpi::writeRegisterBurstMode(uint8_t address, uint8_t numberOfRegisterToWrite, uint8_t *registerValueToWrite, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  SPI.transfer(((triggerMode<<6)&0xC0) | ((0<<5)&0x20) | (address&0x1F));
  for(int i=0;i<numberOfRegisterToWrite;++i) {
    SPI.transfer(registerValueToWrite[i]);
    SPI.transfer(((0<<6)&0xC0) | ((0<<5)&0x20) | ((address+1+i)&0x1F));
    byteCount++;
  }
  digitalWrite(spiChipSelectPin, HIGH);
}

void MV300SensorSpi::readMagneticComponentsWithCrcCheck(uint16_t *magFieldBx, uint16_t *magFieldBy, uint16_t *magFieldBz, uint16_t *temperature, uint8_t *frameCounter, bool *crcErrorDetected, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  uint16_t readbackResult;
  uint8_t sendValue;
  uint8_t readbackValue;
  uint8_t crcReceived, crcChecksum;
  *crcErrorDetected=false;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  sendValue = ((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (0&0x1F);
  readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
  for(int i=0;i<7;++i) {
    sendValue = ((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | ((1+i)&0x1F);
    readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
    readbackValue = (readbackResult>>8)&0xFF;
    readRegistersArray[byteCount]=readbackValue;
    crcReceived = readbackResult&0xFF;
    crcChecksum = crc8(0,readbackValue);
    if (crcChecksum != crcReceived) {
      *crcErrorDetected=true;
    }
    byteCount++;
  }
  digitalWrite(spiChipSelectPin, HIGH);
  *magFieldBx=(readRegistersArray[0]<<4) | ((readRegistersArray[4]&0xF0)>>4);
  *magFieldBy=(readRegistersArray[1]<<4) | (readRegistersArray[4]&0x0F);
  *magFieldBz=(readRegistersArray[2]<<4) | (readRegistersArray[5]&0x0F);
  *temperature=(readRegistersArray[3]<<4) | ((readRegistersArray[5]&0xC0)>>4) | ((readRegistersArray[6]&0x30)>>4);
  *frameCounter=readRegistersArray[6]&0x03;
}

uint8_t MV300SensorSpi::readRegisterWithCrcCheck(uint8_t address, bool *crcErrorDetected, uint8_t triggerMode) {
  uint16_t readbackResult;
  uint8_t readbackValue;
  uint8_t sendValue;
  uint8_t crcReceived, crcChecksum;
  *crcErrorDetected=false;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  sendValue = ((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (address&0x1F);
  readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
  readbackValue = (readbackResult>>8)&0xFF;
  crcReceived = readbackResult&0xFF;
  crcChecksum = crc8(0,readbackValue);
  if (crcChecksum != crcReceived) {
    *crcErrorDetected=true;
  }
  sendValue = ((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (0&0x1F);
  readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
  readbackValue = (readbackResult>>8)&0xFF;
  crcReceived = readbackResult&0xFF;
  crcChecksum = crc8(0,readbackValue);
  if (crcChecksum != crcReceived) {
    *crcErrorDetected=true;
  }
  digitalWrite(spiChipSelectPin, HIGH);
  return readbackValue;
}
 
void MV300SensorSpi::writeRegisterWithCrcCheck(uint8_t address, uint8_t value, bool *crcErrorDetected, uint8_t triggerMode) {
  uint16_t readbackResult;
  uint8_t readbackValue;
  uint8_t sendValue;
  uint8_t crcReceived, crcChecksum;
  *crcErrorDetected=false;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  sendValue = ((triggerMode<<6)&0xC0) | ((0<<5)&0x20) | (address&0x1F);
  readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
  readbackValue = (readbackResult>>8)&0xFF;
  crcReceived = readbackResult&0xFF;
  crcChecksum = crc8(0,readbackValue);
  if (crcChecksum != crcReceived) {
    *crcErrorDetected=true;
  }
  readbackResult = SPI.transfer16(((value<<8)&0xFF00) | (crc8(0,value)&0x00FF));
  readbackValue = (readbackResult>>8)&0xFF;
  crcReceived = readbackResult&0xFF;
  crcChecksum = crc8(0,readbackValue);
  if (crcChecksum != crcReceived) {
    *crcErrorDetected=true;
  }
  sendValue = ((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (0&0x1F);
  readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
  readbackValue = (readbackResult>>8)&0xFF;
  crcReceived = readbackResult&0xFF;
  crcChecksum = crc8(0,readbackValue);
  if (crcChecksum != crcReceived) {
    *crcErrorDetected=true;
  }
  digitalWrite(spiChipSelectPin, HIGH);
}

uint8_t MV300SensorSpi::readRegisterBurstModeWithCrcCheck(uint8_t address, uint8_t numberOfRegisterToRead, uint8_t* readRegisters, bool *crcErrorDetected, uint8_t triggerMode) {
  uint8_t byteCount = 0;
  uint16_t readbackResult;
  uint8_t sendValue;
  uint8_t readbackValue;
  uint8_t crcReceived, crcChecksum;
  *crcErrorDetected=false;
  digitalWrite(spiChipSelectPin, LOW);
  delayMicroseconds(7); //tcsl
  sendValue = ((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | (address&0x1F);
  readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
  for(int i=0;i<numberOfRegisterToRead;++i) {
    sendValue = ((triggerMode<<6)&0xC0) | ((1<<5)&0x20) | ((address+1+i)&0x1F);
    readbackResult = SPI.transfer16(((sendValue<<8)&0xFF00) | (crc8(0, sendValue)&0x00FF));
    readbackValue = (readbackResult>>8)&0xFF;
    readRegisters[byteCount]=readbackValue;
    crcReceived = readbackResult&0xFF;
    crcChecksum = crc8(0,readbackValue);
    if (crcChecksum != crcReceived) {
      *crcErrorDetected=true;
    }
    byteCount++;
  }
  digitalWrite(spiChipSelectPin, HIGH);
  return byteCount;
}
 
void MV300SensorSpi::setClockFrequency(uint32_t speedMaximum) {
  spiMaximumSpeed = speedMaximum;
  SPI.beginTransaction(SPISettings(spiMaximumSpeed, MSBFIRST, spiDataMode));
}
 
void MV300SensorSpi::setDataMode(uint8_t mode) {
  spiDataMode = mode;
  SPI.beginTransaction(SPISettings(spiMaximumSpeed, MSBFIRST, spiDataMode));
}
 
void MV300SensorSpi::setChipSelectPin(uint8_t chipSelectPin) {
  spiChipSelectPin = chipSelectPin;
  pinMode(spiChipSelectPin, OUTPUT);
  digitalWrite(spiChipSelectPin, HIGH);
}

uint32_t MV300SensorSpi::getClockFrequency() {
  return spiMaximumSpeed;
}

uint8_t MV300SensorSpi::getDataMode() {
  return spiDataMode;
}

uint8_t crc8(uint8_t crc, uint8_t data) {
  return crcPolynomialLookupTable[crc^data];
}
 
