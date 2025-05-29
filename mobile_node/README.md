# Design Task 2
Shane Baptist 47484164

## Description
Multithreaded program that creates a shell instance to interface with the CCS811, HTS221 and LPS22HB sensors to measure and log ambient temperature, humidity, pressure and total volatile organic compounds in JSON formatting through the use of the Zephyr JSON API. 

Interfaces with on board RTC using the Zephyr Real-Time Counter API.

The on-board RGB LED will display the current reading of the CCS811 TVOC sensor channel.

## Folder Structure
- p2
    - boards
        - thingy52_nrf52832.overlay
    - src
        - main.c
    - CMakeLists.txt
    - prj.conf
    - README.md
    - sample.yaml

## References
https://docs.zephyrproject.org/latest/index.html

## Instructions
SHELL COMMANDS

rtc
- r - Displays the current time on the RTC
- w [time] - Writes a new time to the RTC

sensor [DID] - Displays the current reading of the sensor/s with the given DID

sample
- s [DID] - Enables continuous sampling for the sensor/s with the provided DID
- p [DID] - Disables continuous sampling for the sensor/s with the provided DID
- w [rate] - Sets the global sampling rate
 
Pressing the on-board button will toggle the sampling setting for the DID of the last interacted sensor.