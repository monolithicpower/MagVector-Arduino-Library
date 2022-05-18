#include <mv300sensor.h>

MV300SensorI2c mv300sensori2c;
uint8_t registerReadbackValue = 0;
uint16_t magFieldBx, magFieldBy, magFieldBz, temperature;
int16_t magFieldBxSigned, magFieldBySigned, magFieldBzSigned, temperatureSigned;
double magFieldBxInMilliTesla, magFieldByInMilliTesla, magFieldBzInMilliTesla, temperatureInDegreeCelsius;
uint8_t frameCounter;
char data[400];

void setup() {
  mv300sensori2c.begin();
  mv300sensori2c.setDeviceAddress(0x14);
  mv300sensori2c.setClockFrequency(400000);
  Serial.begin(115200, SERIAL_8N1);
  while(!Serial){}//wait for serial port to connect. Needed for native USB
  mv300sensori2c.writeRegister(16, 4, 0); //Set TRIGMODE=1 (Trigger measurement after write or read frame), BRANGE=0 (±250mT)
  mv300sensori2c.writeRegister(17, 1, 0); //Set MODE = 1 (Host Controlled Mode)
  for(uint8_t i=0;i<20;++i) {
    registerReadbackValue=mv300sensori2c.readRegister(i, 0);
    sprintf(data, "Reg[%u] = %u", i, registerReadbackValue);
    Serial.println(data);
  }
  mv300sensori2c.readRegister(0, 1); //Read register 0 and trigger a new acquisition by setting 2-bit Trigger to 1
}

void loop() {
  delayMicroseconds(175); //wait for the previous conversion to finish
  mv300sensori2c.readMagneticComponents(&magFieldBx, &magFieldBy, &magFieldBz, &temperature, &frameCounter);
  magFieldBxSigned=twosComplement(magFieldBx,  12);
  magFieldBySigned=twosComplement(magFieldBy,  12);
  magFieldBzSigned=twosComplement(magFieldBz,  12);
  temperatureSigned=twosComplement(temperature, 12);
  magFieldBxInMilliTesla=convertMagneticFieldFromLsbToMilliTesla(magFieldBxSigned, 0);
  magFieldByInMilliTesla=convertMagneticFieldFromLsbToMilliTesla(magFieldBySigned, 0);
  magFieldBzInMilliTesla=convertMagneticFieldFromLsbToMilliTesla(magFieldBzSigned, 0);
  temperatureInDegreeCelsius=convertTemperatureFromLsbToDegreeCelsius(temperatureSigned);
  sprintf(data, "Bx = % 8.3f mT, By = % 8.3f mT, Bz = % 8.3f mT, Temperature = % 6.1f °C, frame = %u", magFieldBxInMilliTesla, magFieldByInMilliTesla, magFieldBzInMilliTesla, temperatureInDegreeCelsius, frameCounter);
  Serial.println(data);
}