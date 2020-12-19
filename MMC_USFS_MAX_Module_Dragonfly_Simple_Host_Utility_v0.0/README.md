# USFSMAX MMC Module Dragonfly Test Sketch

This interconnect information is for operating the [USFSMAX motion coprocessor board](https://cdn.tindiemedia.com/images/resize/S4Os6lUdoJOFuyZHntYHBDiDCUk=/p/full-fit-in/1782x1336/i/44691/products/2020-02-03T20%3A51%3A19.878Z-USFSMAX.top.jpg) with the Tlera Dragonfly [STM32L476 development board](https://cdn.tindiemedia.com/images/resize/c3IuiwcFMGzoMgmNkZ_doDvJiuU=/p/full-fit-in/1782x1336/i/32456/products/2017-01-04T23%3A18%3A53.356Z-2016-05-14T16%2031%2059.360Z-Layout.jpg.855x570_q85_pad_rcrop.jpg). The default configuration of the sketch is set up to use the "Wire" instance of the TWI I2C library and pin "8" as the data ready (DRDY) interrupt. The necessary connections between the two boards are:

## Prototype Breadboard
|USFSMAX Pin|Dragonfly Pin|
|:---------:|:-----------:|
|   3V3     |     3V3     |
|   GND     |     GND     |
|   SCL     |    21(SCL)  |
|   SDA     |    20(SDA)  |
|   INT     |      8      |
|   Wake    |      30     |

## USFS "Piggybacked" onto Development Board
|USFSMAX Pin|Dragonfly Pin|
|:---------:|:-----------:|
|   3V3     |      31     |
|   GND     |      13     |
|   SCL     |    21(SCL)  |
|   SDA     |    20(SDA)  |
|   INT     |      8      |
|   Wake    |      30     |

![alt text](https://user-images.githubusercontent.com/5760946/102422567-eea29a00-3fbb-11eb-8a9e-fae31dcb2a69.JPG)

The USFSMAX board can also be used with either of the other two I2C ports and a different DRDY interrupt but the "INT_PIN" and "SENSOR_0_WIRE_INSTANCE" definitions in the "config.h" tab would need updating to reflect these changes.

## Building/Uploading/Running the Test Sketch

The Sketch is built and uploaded using the [STM32L4 core for the Arduino IDE](https://github.com/GrumpyOldPizza/arduino-STM32L4):
* Install the STM32L4 core as directed in the repository
* Download the STM32L4 USFSMAX Simple Host Utility sketch from this repository and open it with the Arduino IDE
* Go into the "Tools->Board:" menu entry of the IDE and select the "Dragonfly-L476RE" entry
* Plug the Dragonfly development board into an available USB port on your PC. If this is the first time your Dragonfly board is being used, you might want to consult the [Dragonfly Wiki](https://github.com/kriswiner/Dragonfly/wiki)
* Build/Upload the sketch as you would with any other Arduino board
* Once the upload is complete, power cycle the Dragonfly/USFSMAX by unplugging and replugging the USB cable
* Open the Arduino serial monitor at 115200 baud. You should see some startup messages and then the calibration data scroll across the serial monitor
* Send a "1" over the serial monitor; you will be prompted to go a gyro cal. Make sure the USFSMAX module is sitting still
* Send a "1" over the serial monitor; you should see AHRS and sensor continuously scrolling across the serial monitor
