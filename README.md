# Device BME680 Service

## About
The EdgeX Device BME680 Service is developed to communicate to BME680 sensor connected to the I2C bus in an EdgeX deployment

## Supported Boards:
* NXP ls1012a frwy - ARM64 bit
* Raspberry Pi 3B(+) - ARM64 bit

## Dependencies:

The Device BME680 service based on [device-grove-c](https://github.com/edgexfoundry/device-grove-c)
is developed using BME680_driver library.
The repository can be found on git at [BME680_driver](https://github.com/BoschSensortec/BME680_driver). 

## Build Instruction:

1. Check out device-bme680-c available at [device-bme680-c](https://github.com/YustasSwamp/device-bme680-c)

2. Build/install device-c-sdk library and headers by using the following command
```
sh> ./scripts/build_deps.sh
```
3. Build the device service
```
sh> ./scripts/build.sh
```
By default, the configuration and profile file used by the service are available in __'res'__ folder.

## Configuration
1. Port number specified in the configuration.toml
2. /dev/i2c-0 is hardcoded in src/c/main.c file since in NXP board it is i2c-0. For Raspberry PI, it might be i2c-1.

**Note:** 
1. Make sure that i2c_dev module is loaded and /dev/i2c-X file is present
2. On Raspberry PI, make sure that i2c_arm=on is set. This enables i2c-1 device, required for communication between Grove PI & Raspberry PI boards.














