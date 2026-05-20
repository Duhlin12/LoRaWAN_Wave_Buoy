# LoRaWAN Wave-Measuring Buoy (Heltec CubeCell HTCC-AB02)

This repository contains Arduino firmware for an autonomous wave-measuring telemetry buoy, specifically written and optimized for the **Heltec CubeCell HTCC-AB02** (ASR6502) platform.

The device samples acceleration and orientation data to estimate wave dynamics, collects GPS coordinates, reads sea temperature, and transmits the packaged payload over an EU868 LoRaWAN network using **ABP (Activation by Personalization)**.

## Key Features

* **Optimized for CubeCell HTCC-AB02:** Built utilizing the native `LoRaWan_APP.h` library for the ASR6502 chip, taking full advantage of its multi-GPIO power management and deep sleep capabilities.
* **Wave Motion Analysis:** Utilizes an **MPU6050** IMU to measure raw acceleration. Applies tilt compensation (using pitch/roll angles) to compute true vertical acceleration and integrates it to estimate significant wave height ($H_s$) and maximum wave height ($H_{max}$).
* **RAM-Based Land Calibration:** Performs a 100-sample sensor calibration during `setup()` to lock initial land offsets into RAM. *Note: The device needs a manual reset/cold start on dry land before deployment.* Continuous sensor drift is eliminated during runtime using a low-pass filter.
* **High-Precision GPS:** Interfaces with a GPS module via `TinyGPS++`. Implements a strict quality gate requiring an HDOP less than or equal to 2.0 and at least 8 connected satellites before locking location.
* **Sea Temperature Sensing:** Reads water temperature using a **DS18B20** sensor via OneWire.
* **Low-Power Optimization:** Strictly manages hardware power rails by toggling `Vext` and a dedicated `SENSOR_POWER_PIN` (GPIO2) to control and cut power directly to peripheral sensors during deep sleep cycles.
* **Efficient Payload Packing:** Packs battery level, wave metrics, temperature, and high-precision coordinates into a compact 14-byte array to minimize time-on-air.

## Hardware Components

* **MCU/LoRa Board:** Heltec CubeCell HTCC-AB02
* **IMU:** MPU6050 (Connected via I2C)
* **Temperature Sensor:** DS18B20 (Connected via OneWire on GPIO1)
* **GPS Module:** Standard NMEA GPS Module (Connected via Serial1)

## Setup & Configuration

Before deploying or compiling the code, make sure to replace the placeholder keys in the configuration section with your own LoRaWAN ABP credentials from your network provider (e.g., The Things Network):

```cpp
/* --- LORAWAN ABP KEYS --- */
uint32_t devAddr = ( uint32_t )0xYOUR_DEV_ADDR; 
uint8_t nwkSKey[] = { 0xXX, 0xXX, ... }; 
uint8_t appSKey[] = { 0xXX, 0xXX, ... };