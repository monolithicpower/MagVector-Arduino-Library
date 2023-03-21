[![Compile Examples](https://github.com/monolithicpower/MagVector-Arduino-Library/actions/workflows/compile-examples.yml/badge.svg)](https://github.com/monolithicpower/MagVector-Arduino-Library/actions/workflows/compile-examples.yml)
# MagVector-Arduino-Library
Arduino library for the MPS MagVector 3D magnetic sensor.

## About
The MV300 detects the direction and strength of the 3D (X, Y, Z) magnetic field. The signal from Hall sensors is amplified and converted to digital signal. The magnetic field range can be scaled from ±125mT to ±250mT. 

An on-chip temperature sensor provides the information about the chip temperature, which is helpful for off-chip user calibration. 

The system can operate in host controlled mode, ASC (Auto Sampling Cycle) mode and full-speed mode. Fast data acquisition of AC magnetic fields updates at the frequency of up to 20kHz. 

The communication with MV300 can be done either via I2C or SPI interface, depending on the selected version.

## Supported sensors
Supports all MagVector 3D magnetic sensors from [Monolithic Power Systems](https://www.monolithicpower.com/).

| Applications | Part Numbers |
| ------------| ------------ |
| 3D Magnetic Sensor with Digital Output | MV300 |

## License
Written by Mathieu Kaelin for Monolithic Power Systems (MPS).
MIT license, all text above must be included in any redistribution.

## Connections
### Power supply
| Arduino  | MagVector |
| -------- | --------- |
| GND      | GND       |
| **+3.3V** (Not 5V)| VDD  |

| Warning |
| :-------: |
| Unlike some Arduino & Genuino boards, the **MagVector runs at 3.3V**. Even if the I/O can tolerate 5V, check that the voltage applied to VDD is at 3.3V. **Applying a voltages higher than 3.3V to the VDD pin could damage the sensor**.|

### Serial communication
MagVector sensors have different communication interfaces available. The MV300 is available with either a I2C or a SPI communication interface.

#### I2C (2-wire interface)
| Arduino  | MagVector |
| -------- | --------- |
| SCL      | SCL       |
| SDA      | SDA       |

Do not forget to connect SA1 and SA2 pins to define the I2C device address.

#### SPI (4-wire interface)
| Arduino         | MagVector |
| --------------- | --------- |
| COPI (aka MOSI) | COPI      |
| CIPO (aka MISO) | CIPO      |
| SCK             | SCLK      |
| any available digital pin (default: pin 7) | /CS |

## Setup
Install the library directly from within the Arduino IDE by using the Library Manager (Sketch => Include Library => Manage Libraries...).

It is also possible to import the library Zip file (check release tab) from the Arduino IDE (Sketch => Include Library => Add .ZIP Library...).

The library can also be manually installed by copying the MagVector library folder in your arduinosketchfolder/libraries/ folder. You may need to create the libraries subfolder if its your first library. Restart the IDE to see the library.

Check this tutorial on Arduino library installation for more information:
* [All About Arduino Libraries](http://learn.adafruit.com/adafruit-all-about-arduino-libraries-install-use)
