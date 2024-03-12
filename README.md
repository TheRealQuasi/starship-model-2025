# Starship Project

This project involves programming a Teensy 4.1 microcontroller using PlatformIO in Visual Studio Code.

## Setup

1. Install Visual Studio Code.
2. Install the PlatformIO extension in Visual Studio Code.
3. Open the `microcontroller` folder in Visual Studio Code. PlatformIO should automatically detect all necessary files.

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

Please ensure these libraries are installed with platformIO before building the project.
