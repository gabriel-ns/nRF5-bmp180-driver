# nRF5 Sensor Library
## About

This library was developed using the Nordic Semiconductor's NRF5 SDK version 12.3. It should work with all the 12.X versions.

It's an synchronous library based on the Nordic's nrf_drv_twi library available in the SDK, in the folder $(SDK_ROOT)/components/libraries/twi.

## Changelog

__V1.0.0__
* Fully operational implementation for HTU21D sensor
* Fully operational implementation for BMP180 sensor 
* Basic implementation for TSL2561 sensor. No support for custom integration time, interrupts and CL package lux calculation.

## Available Sensors
* BMP180 Pressure and Temperature sensor (compatible with BMP085)
* HTU21D High Precision Temperature and Humidity sensor
* TSL2561 Light-to-Digital converter

## Sample projects

The sample projects were developed with the [Eclipse Neon 3 IDE](https://www.eclipse.org/neon/) and GNU ARM GCC. If you need a step by step environment setup, just check this [Nordic's tutorial](https://devzone.nordicsemi.com/tutorials/7/). 

__IMPORTANT__: On these examples, the SDK must be one folder above the repository!

These examples simply call the necessary methods to get readings for each sensor. With a debugger and setting breakpoints after the readings you will be able to check the data.