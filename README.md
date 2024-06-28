# RP2040 enviro logger
Logger for environmental parameters based on the Arduino Nano RP2040 Connect

## Goal
Record basic environmental parameters such as temperature, relative humidity and air pressure as well as short samples of sound and accelerometer readings.

## Methods
[Arduino Nano RP2040 Connect](https://docs.arduino.cc/hardware/nano-rp2040-connect/) has a built-in microphone and combined accelerometer and gyroscope with temperature sensor. It's also capable of WiFi and BLE communication. To further expand the capabilities of the device I connected the BME280 temperature, relative humidity and air pressure sensor mounted on the [Pimoroni breakout board](https://shop.pimoroni.com/products/bme280-breakout).

## Connections
| Arduino Nano | BME280 |
| ------------ | ------ |
| 3V3          | 2-6V   |
| GND          | GND    |
| A4           | SDA    |
| A5           | SCL    |

## Links
[Arduino Nano RP2040 cheat sheet](https://docs.arduino.cc/tutorials/nano-rp2040-connect/rp2040-01-technical-reference)

[Read audio data on Arduino Nano RP2040](https://docs.arduino.cc/tutorials/nano-rp2040-connect/rp2040-microphone-basics)

## TODO
* Explain available options
* Consider replacing error messages with error codes
