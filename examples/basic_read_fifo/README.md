# basic_read_fifo

This application demonstrates how to retrieve data from the FIFO using the basic driver.
FIFO threshold is set to 1 packet and configured on INT1. Accel and gyro ODR are set to 50 Hz.

## Command interface

This application allows the following command to be sent through UART:
* `s`: to toggle print data in SI unit (defaults to enable).
* `l`: to toggle print data in LSB (defaults to disable).
* `a`: to toggle enable/disable accelerometer (defaults to enable).
* `g`: to toggle enable/disable gyroscope (defaults to enable).
* `i`: to toggle enable/disable FIFO high-resolution mode (defaults to disable).
* `p`: to select low-noise or low-power sensor mode, common to accelerometer and gyroscope (defaults to low-noise).
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

* When print in SI unit is enabled:
```
SI <timestamp> us Accel: <acc_x> <acc_y> <acc_z> g Gyro: <gyr_x> <gyr_y> <gyr_z> dps Temp: <temp> degC FIFO time: <fifo_time> us
```
* When print in LSB is enabled:
```
LSB <timestamp> us Accel: <raw_acc_x> <raw_acc_y> <raw_acc_z> Gyro: <raw_gyr_x> <raw_gyr_y> <raw_gyr_z> Temp: <raw_temp> FIFO time: <fifo_time> us
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired
* `<raw_acc_x|y|z>`: Raw accelerometer value
* `<acc_x|y|z>`: Raw accelerometer value converted in g
* `<raw_gyr_x|y|z>`: Raw gyroscope value
* `<gyr_x|y|z>`: Raw gyroscope value converted in dps
* `<raw_temp>`: Raw temperature value
* `<temp>`: Raw temperature value converted in °C
* `<fifo_time>`: 16-bit timestamp field in us as read in FIFO, this is time at which ICM samples current FIFO packet, unit being 1 us or 16 us depending on ICM configuration (defaults to 1 us). Rolls over 65535.

### Example of output

```
[I] ###
[I] ### Example Read FIFO (using basic API)
[I] ###
[I] SI        7262 us   Accel:   -0.02     0.01     1.01 g   Gyro:       -        -        -       Temp:  27.00 degC   FIFO Time:  1718 us
[I] SI       27162 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.12     0.37 dps   Temp:  27.50 degC   FIFO Time: 21718 us
[I] SI       47045 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.06     0.24 dps   Temp:  27.50 degC   FIFO Time: 41718 us
[I] SI       66929 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32    -0.06     0.06 dps   Temp:  27.50 degC   FIFO Time: 61718 us
[I] SI       86813 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32    -0.06     0.12 dps   Temp:  27.00 degC   FIFO Time: 16182 us
[I] SI      106696 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.12 dps   Temp:  27.00 degC   FIFO Time: 36182 us
[I] SI      126580 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32    -0.06     0.12 dps   Temp:  27.00 degC   FIFO Time: 56182 us
[I] SI      146463 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 10646 us
[I] SI      166347 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 30646 us
[I] SI      186231 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.18 dps   Temp:  27.00 degC   FIFO Time: 50646 us
[I] SI      206114 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time:  5110 us
[I] SI      225998 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.12 dps   Temp:  27.00 degC   FIFO Time: 25110 us
[I] SI      245881 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 45110 us
[I] SI      265765 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 65110 us
[I] SI      285649 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 19574 us
[I] SI      305532 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.32    -0.06     0.06 dps   Temp:  27.50 degC   FIFO Time: 39574 us
[I] Disabling SI print.
[I] Enabling LSB print.
[I] LSB    1916104 us   Accel:    -168       57     8299     Gyro:     -38        0        2       Temp:      5        FIFO Time: 21174 us
[I] LSB    1935988 us   Accel:    -163       57     8305     Gyro:     -38        0        2       Temp:      4        FIFO Time: 41174 us
[I] LSB    1955871 us   Accel:    -165       54     8301     Gyro:     -38       -2        2       Temp:      5        FIFO Time: 61174 us
[I] LSB    1975755 us   Accel:    -165       56     8298     Gyro:     -38        1        2       Temp:      4        FIFO Time: 15638 us
[I] LSB    1995639 us   Accel:    -171       52     8303     Gyro:     -38        1        2       Temp:      5        FIFO Time: 35638 us
[I] LSB    2015522 us   Accel:    -178       50     8304     Gyro:     -38        0        2       Temp:      4        FIFO Time: 55638 us
[I] LSB    2035406 us   Accel:    -176       49     8303     Gyro:     -38        1        2       Temp:      4        FIFO Time: 10102 us
[I] LSB    2055290 us   Accel:    -173       52     8304     Gyro:     -38       -1        2       Temp:      5        FIFO Time: 30102 us
[I] LSB    2075173 us   Accel:    -171       58     8302     Gyro:     -38       -2        2       Temp:      5        FIFO Time: 50102 us
[I] LSB    2095057 us   Accel:    -166       56     8301     Gyro:     -38        0        2       Temp:      5        FIFO Time:  4566 us
[I] LSB    2114940 us   Accel:    -168       49     8306     Gyro:     -38        0        2       Temp:      5        FIFO Time: 24566 us
[I] LSB    2134824 us   Accel:    -175       50     8305     Gyro:     -37       -1        2       Temp:      4        FIFO Time: 44566 us

```

