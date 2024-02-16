# eMD software for ICM-45686

_eMD_ stands for _embedded Motion Driver_.

## Summary

The eMD software for ICM-45686 consists in a driver and a set of examples to showcase our IMU's capabilities. It targets microcontroller-based application and is coded in C language.

## Content

The `icm45686/` folder contains the drivers for the IMU.

The `examples/` folder contains:
* A set of examples showcasing the IMU capabilities: `examples/*/`. Each sub-folder contains a README describing the application. 
* The hardware abstraction layer definition: `system_interface.h`. 

## Building the code

A `CMakeLists.txt` file is provided as an example on how to build.

Open a terminal and go to the project root folder, then execute the following commands:

```
cmake -S . -B build/
cmake --build build/
```
