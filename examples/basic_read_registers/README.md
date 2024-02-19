# basic_read_registers

This application demonstrates how to retrieve data from the ICM sensor data registers using the basic driver. DRDY interrupt is configured on INT1. Accel and gyro ODR are set to 50 Hz.

## Command interface

This application allows the following command to be sent through UART:
* `s`: to toggle print data in SI unit (defaults to enable).
* `l`: to toggle print data in LSB (defaults to disable).
* `a`: to toggle enable/disable accel (defaults to enable).
* `g`: to toggle enable/disable gyro (defaults to enable).
* `p`: to select low-noise or low-power sensor mode, common to accel and gyro (defaults to low-noise).
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

* When print in SI unit is enabled :
```
SI <timestamp> us Accel: <acc_x> <acc_y> <acc_z> g Gyro: <gyr_x> <gyr_y> <gyr_z> dps Temp: <temp> degC
```
* When print in LSB is enabled :
```
LSB <timestamp> us Accel: <raw_acc_x> <raw_acc_y> <raw_acc_z> Gyro: <raw_gyr_x> <raw_gyr_y> <raw_gyr_z> Temp: <raw_temp>
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired
* `<raw_acc_x|y|z>`: Raw accelerometer value
* `<acc_x|y|z>`: Raw accelerometer value converted in g
* `<raw_gyr_x|y|z>`: Raw gyroscope value
* `<gyr_x|y|z>`: Raw gyroscope value converted in dps
* `<raw_temp>`: Raw temperature value
* `<temp>`: Raw temperature value converted in °C

### Example of output

```
[I] ###
[I] ### Example Read registers (using basic API)
[I] ###
[I] SI        7116 us   Accel:   -0.02     0.00     1.02 g   Gyro:       -        -        -       Temp:  27.63 degC
[I] SI       27014 us   Accel:   -0.02     0.00     1.02 g   Gyro:   -2.20     0.12     0.37 dps   Temp:  27.31 degC
[I] SI       46898 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.06     0.24 dps   Temp:  27.50 degC
[I] SI       66781 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.12 dps   Temp:  27.63 degC
[I] SI       86665 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.50 degC
[I] SI      106549 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.69 degC
[I] SI      126432 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20    -0.06     0.12 dps   Temp:  27.56 degC
[I] SI      146316 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.69 degC
[I] SI      166200 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.44 degC
[I] SI      186084 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.12 dps   Temp:  27.50 degC
[I] SI      205967 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.56 degC
[I] SI      225851 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.06 dps   Temp:  27.63 degC
[I] SI      245735 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.06 dps   Temp:  27.44 degC
[I] SI      265618 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.44 degC
[I] SI      285502 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.56 degC
[I] SI      305386 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.56 degC
[I] SI      325269 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.63 degC
[I] SI      345153 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.75 degC
[I] SI      365037 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.69 degC
[I] SI      384921 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.50 degC
[I] SI      404804 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.63 degC
[I] SI      424688 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.06 dps   Temp:  27.56 degC
[I] SI      444572 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.81 degC
[I] SI      464455 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.56 degC
[I] LSB    1438757 us   Accel:    -157       47     8307     Gyro:     -37       -1        2       Temp:    344
[I] LSB    1458641 us   Accel:    -159       54     8306     Gyro:     -36        1        2       Temp:    328
[I] LSB    1478524 us   Accel:    -136       62     8312     Gyro:     -37        0        2       Temp:    328
[I] LSB    1498408 us   Accel:    -148       57     8306     Gyro:     -37       -3        2       Temp:    360
[I] LSB    1518292 us   Accel:    -160       53     8303     Gyro:     -36        1        2       Temp:    336
[I] LSB    1538175 us   Accel:    -148       48     8309     Gyro:     -36        0        2       Temp:    344
[I] LSB    1558059 us   Accel:    -150       53     8304     Gyro:     -37       -1        2       Temp:    328
[I] LSB    1577943 us   Accel:    -144       55     8306     Gyro:     -36        1        2       Temp:    352
[I] LSB    1597826 us   Accel:    -143       55     8310     Gyro:     -36       -1        2       Temp:    336
[I] LSB    1617710 us   Accel:    -152       51     8309     Gyro:     -36       -2        2       Temp:    328
[I] LSB    1637594 us   Accel:    -151       55     8307     Gyro:     -36        0        2       Temp:    336
[I] LSB    1657478 us   Accel:    -146       54     8307     Gyro:     -36        0        2       Temp:    288
[I] LSB    1677361 us   Accel:    -148       48     8306     Gyro:     -36       -1        2       Temp:    328
[I] LSB    1697245 us   Accel:    -154       50     8309     Gyro:     -37        0        2       Temp:    320
[I] LSB    1717129 us   Accel:    -151       54     8308     Gyro:     -37       -1        2       Temp:    328
[I] LSB    1737012 us   Accel:    -147       54     8308     Gyro:     -37       -1        1       Temp:    344
[I] LSB    1756896 us   Accel:    -146       55     8311     Gyro:     -37        0        2       Temp:    336
[I] LSB    1776780 us   Accel:    -148       54     8308     Gyro:     -37        0        2       Temp:    344
```

