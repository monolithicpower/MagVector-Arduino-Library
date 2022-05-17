#ifndef MV300SENSOR_H
#define MV300SENSOR_H


#if (ARDUINO >= 100)
     #include "Arduino.h"
#else
     #include "WProgram.h"
#endif
#include <SPI.h>
#include <Wire.h>

class MV300SensorI2c {
public:
  MV300SensorI2c();
  void begin();
  void end();
  void setDeviceAddress(uint8_t deviceAddress);
  uint8_t getDeviceAddress();
  void setClockFrequency(uint32_t clockFrequency);
  uint32_t getClockFrequency();
  void readMagneticComponents(uint16_t *magFieldBx, uint16_t *magFieldBy, uint16_t *magFieldBz, uint16_t *temperature, uint8_t *frameCounter);
  uint8_t readRegisterQuickReadMode(uint8_t numberOfRegisterToRead, uint8_t *readRegisters);
  uint8_t readRegisterBurstMode(uint8_t address, uint8_t numberOfRegisterToRead, uint8_t *readRegisters, uint8_t triggerMode=0);
  void writeRegisterBurstMode(uint8_t address, uint8_t numberOfRegisterToWrite, uint8_t *registerValueToWrite, uint8_t triggerMode=0);
  uint8_t readRegister(uint8_t address, uint8_t triggerMode=0);
  void writeRegister(uint8_t address, uint8_t value, uint8_t triggerMode=0);
  void readMagneticComponentsWithCrcCheck(uint16_t *magFieldBx, uint16_t *magFieldBy, uint16_t *magFieldBz, uint16_t *temperature, uint8_t *frameCounter, bool *crcErrorDetected);
  uint8_t readRegisterQuickReadModeWithCrcCheck(uint8_t numberOfRegisterToRead, uint8_t* readRegisters, bool *crcErrorDetected);
  uint8_t readRegisterBurstModeWithCrcCheck(uint8_t address, uint8_t numberOfRegisterToRead, uint8_t* readRegisters, bool *crcErrorDetected, uint8_t triggerMode=0);
  uint8_t readRegisterWithCrcCheck(uint8_t address, bool *crcErrorDetected, uint8_t triggerMode=0);
  void writeRegisterWithCrcCheck(uint8_t address, uint8_t value, uint8_t triggerMode=0);  
private:
  uint8_t i2cAddress;
  uint32_t i2cClockFrequency;
  uint8_t readRegistersArray[32];
  bool debugModeEnabled;
  uint8_t crc8(uint8_t crc, uint8_t data);
};

class MV300SensorSpi {
public:
  MV300SensorSpi();
  void begin(uint8_t chipSelectPin);
  void begin(int32_t sclkFrequency, uint8_t mode, uint8_t chipSelectPin);
  void end();
  void readMagneticComponents(uint16_t *magFieldBx, uint16_t *magFieldBy, uint16_t *magFieldBz, uint16_t *temperature, uint8_t *frameCounter, uint8_t triggerMode=0);
  uint8_t readRegister(uint8_t address, uint8_t triggerMode=0);
  void writeRegister(uint8_t address, uint8_t value, uint8_t triggerMode=0);
  uint8_t readRegisterQuickReadMode(uint8_t numberOfRegisterToRead, uint8_t* readRegisters, uint8_t triggerMode=0);
  uint8_t readRegisterQuickReadModeWithCrcCheck(uint8_t numberOfRegisterToRead, uint8_t* readRegisters, bool *crcErrorDetected, uint8_t triggerMode=0);
  uint8_t readRegisterWithCrcCheck(uint8_t address, bool *crcErrorDetected, uint8_t triggerMode=0);
  void writeRegisterWithCrcCheck(uint8_t address, uint8_t value, bool *crcErrorDetected, uint8_t triggerMode=0);
  void setClockFrequency(uint32_t speedMaximum);
  uint32_t getClockFrequency();
  void setDataMode(uint8_t mode);
  uint8_t getDataMode();
  void setChipSelectPin(uint8_t chipSelectPin);
private:
  uint32_t spiMaximumSpeed;
  uint8_t spiDataMode;
  uint8_t spiChipSelectPin;
  uint8_t readRegistersArray[32];
  bool debugModeEnabled;
  uint8_t crc8(uint8_t crc, uint8_t data);
};

#endif //MV300SENSOR_H