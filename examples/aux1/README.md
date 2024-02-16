# aux1

This application demonstrates how to retrieve OIS data from AUX1 interface using only AUX driver. It does not need any configuration from UI interface, which can remain undriven.
It gathers accelerometer and/or gyroscope data when data ready fires INT2.

## Specific wiring

### If IMU EVB is used

To allow IMU EVB AUX ports to be driven by SmartMotion board MCU, IMU EVB must not be plugged on SmartMotion CN1 but through wires instead :

| **IMU pin**     | **SmartMotion pin** | **IMU EVB pin** |
| :-----:         | :----:              | :-----:         |
| AUX1_CS         | J4.1                | CN2.9           |
| AUX1_SCLK       | J4.2                | CN2.7           |
| AUX1_SDIO       | J4.3                | CN2.8           |
| AUX1_SDO        | J4.4                | CN2.5           |
| INT2            | J4.6                | CN3.5           |
| GND             | J4.7                | CN2.10          |

| **Function**    | **SmartMotion pin** | **IMU EVB pin** |
| :-----:         | :----:              | :-----:         |
| 5V              | CN1.19              | CN2.4           |

Please also make sure EVB jumper J1 closes 3-5 and 4-6

\if icm45631

As ICM-45631 is triple interface by default, only SPI3 is supported on AUX1 interface at POR. UI link must then be set up to disable AUX2 prior to any AUX1 operation, hence making SPI4 available on AUX1 interface.

| **IMU pin**     | **SmartMotion pin** | **IMU EVB pin** |
| :-----:         | :----:              | :-----:         |
| AP_SDA          | CN2.3               | CN1.18          |
| AP_SCL          | CN2.5               | CN1.16          |

Please also make sure SmartMotion jumper J1 closes 5-6 and 7-8 (`Mag I2C` configuration)

\endif

### If IMU DB is used

To allow IMU DB AUX ports to be driven by SmartMotion board MCU, IMU DB must not be plugged on SmartMotion CN4/CN5 but through wires instead :

| **IMU pin**     | **SmartMotion pin** | **IMU DB pin** |
| :-----:         | :----:              | :-----:        |
| AUX1_CS         | J4.1                | CN4.4          |
| AUX1_SCLK       | J4.2                | CN4.5          |
| AUX1_SDIO       | J4.3                | CN4.3          |
| AUX1_SDO        | J4.4                | CN4.2          |
| INT2            | J4.6                | CN5.2          |
| GND             | J4.7                | CN4.10         |
| VDD             | CN5.10              | CN5.10         |
| VDDIO           | CN4.9               | CN4.9          |


\if icm45631

As ICM-45631 is triple interface by default, only SPI3 is supported on AUX1 interface at POR. UI link must then be set up to disable AUX2 prior to any AUX1 operation, hence making SPI4 available on AUX1 interface.

| **IMU pin**     | **SmartMotion pin** | **IMU DB pin** |
| :-----:         | :----:              | :-----:        |
| AP_SDA          | CN2.3               | CN5.4          |
| AP_SCL          | CN2.5               | CN5.6          |

Please also make sure SmartMotion jumper J1 closes 5-6 and 7-8 (`Mag I2C` configuration)

\endif

## Command interface

This application allows the following command to be sent through UART:
* `a`: to enable/disable accel (defaults to enable).
* `g`: to enable/disable gyro (defaults to enable).
* `s`: to either print all data at OIS data rate or only average value of all sensor data gathered during last timeframe every second (defaults to print average value).
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

* When statistics are enabled (default):
```
<timestamp> us   Average Accel: <acc_x> <acc_y> <acc_z> g   Average Gyro: <gyr_x> <gyr_y> <gyr_z> dps
```
* When statistics are disabled and all data are printed:
```
<timestamp> us   Accel        : <acc_x> <acc_y> <acc_z> g   Gyro        : <gyr_x> <gyr_y> <gyr_z> dps
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT2 fired.
* `<acc_x|y|z>`: Raw accelerometer value converted in g.
* `<gyr_x|y|z>`: Raw gyroscope value converted in dps.

**When statistics are disabled and all data are printed through `s` keypress, UART terminal might not be able to follow OIS high data rate, so data printed might be truncated**

### Example of output

```
[I] ###
[I] ### Example AUX1
[I] ###
[I]     768721 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    1.66     0.16    -2.22 dps
[I]    1530402 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05    -0.00 dps
[I]    2290680 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05     0.03 dps
[I]    3051270 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.04     0.03 dps
[I]    3811393 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05     0.04 dps
[I]    4571516 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.33     0.05     0.04 dps
[I]    5331639 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.04     0.04 dps
[I]    6091762 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05     0.04 dps
[I]    6852041 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05     0.04 dps
[I]    7612787 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05     0.04 dps
[I]    8373222 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05     0.04 dps
[I]    9133656 us   Average Accel:   -0.59    -0.56    -0.58 g   Average Gyro:    0.32     0.05     0.05 dps
[I] #
[I] # Current configuration
[I] #
[I] # Accel: Enabled
[I] # Gyro: Enabled
[I] # Stats: OFF
[I] #
[I]   24685023 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.37     0.37    -0.06 dps
[I]   24685335 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.24    -0.06    -0.37 dps
[I]   24685646 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.31     0.24     0.12 dps
[I]   24685958 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.24     0.24    -0.06 dps
[I]   24686269 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.24     0.12    -0.18 dps
[I]   24686581 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.31    -0.18    -0.49 dps
[I]   24686892 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.49     0.12     0.06 dps
[I]   24687204 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.12     0.06    -0.18 dps
[I]   24687515 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.24     0.12     0.06 dps
[I]   24687827 us   Accel        :   -0.59    -0.56    -0.58 g   Gyro        :    0.49    -0.06    -0.18 dps
```

