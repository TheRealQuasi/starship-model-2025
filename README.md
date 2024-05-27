# Starship model Project 2024

Modelling and control of an electric reusable rocket
----------------------------------------------------

This project is part of a bachelor theisis at Chalmers University of Technology, department of electrical engineering ([Full report](/Bachelor_thesis_EENX16_24_50_Starship.pdf)
).

It is a continuation of a previous bachlor thesis from 2023, with the goal of buidling a propeller driven rocket model, immitating the control of SpaceX starships second stage.

The project involves programming a Teensy 4.1 microcontroller using PlatformIO in Visual Studio Code.

## Setup

1. Install Visual Studio Code.
2. Install the PlatformIO extension in Visual Studio Code.
3. Open the `rocket` folder in Visual Studio Code. PlatformIO should automatically detect all necessary files.

## Building and Programming

1. Connect the Teensy 4.1 microcontroller to your PC via USB.
2. Build the project by clicking on the checkmark icon in the bottom left corner of the PlatformIO interface.
3. Program the microcontroller by clicking on the arrow icon next to the checkmark.

## Dependencies

This project depends on the following libraries:

- `seeed-studio/Grove IMU 9DOF@^1.0.0`
- `pkourany/MPU6050@^1.0.3`
- `mbed-syundo0730/I2Cdev@0.0.0+sha.3aa973ebe3e5`
- `infineon/XENSIV Digital Pressure Sensor@^1.0.0`

Please ensure these libraries are installed with platformIO before building the project. They should however be installed automatically when opening the project.
