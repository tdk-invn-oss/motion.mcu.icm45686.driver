# adv_read_fifo

This application demonstrates how to retrieve data from the FIFO using the advanced driver. FIFO threshold is set to 1 packet and configured on INT1. Accel and gyro ODR are set to 50 Hz.

## Command interface

This application allows the following command to be sent through UART:
* `s`: to toggle print data in SI unit (defaults to enable).
* `l`: to toggle print data in LSB (defaults to disable).
* `a`: to toggle enable/disable accel (defaults to enable).
* `g`: to toggle enable/disable gyro (defaults to enable).
* `i`: to toggle enable/disable FIFO high-resolution mode (defaults to disable).
* `p`: to select low-noise or low-power sensor mode, common to accel and gyro (defaults to low-noise).
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

* When print in SI unit is enabled :
```
SI <timestamp> us Accel: <acc_x> <acc_y> <acc_z> g Gyro: <gyr_x> <gyr_y> <gyr_z> dps Temp: <temp> degC FIFO time: <fifo_time> us
```
* When print in LSB is enabled :
```
LSB <timestamp> Accel: <raw_acc_x> <raw_acc_y> <raw_acc_z> Gyro: <raw_gyr_x> <raw_gyr_y> <raw_gyr_z> Temp: <raw_temp> FIFO time: <fifo_time> us
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `<raw_acc_x|y|z>`: Raw accel value.
* `<acc_x|y|z>`: Raw accel value converted in g.
* `<raw_gyr_x|y|z>`: Raw gyro value.
* `<gyr_x|y|z>`: Raw gyro value converted in dps.
* `<raw_temp>`: Raw temp value.
* `<temp>`: Raw temp value converted in °C.
* `<fifo_time>`: 16-bit timestamp field in us as read in FIFO.

### Example of output

```
[I] ###
[I] ### Example Read FIFO (using advanced API)
[I] ###
[I] SI        7346 us   Accel:       -        -        -     Gyro:       -        -        -       Temp:  27.50 degC   FIFO Time:  1718 us
[I] SI       27245 us   Accel:   -0.02     0.01     1.02 g   Gyro:       -        -        -       Temp:  27.50 degC   FIFO Time: 21718 us
[I] SI       47128 us   Accel:   -0.02     0.01     1.01 g   Gyro:       -        -        -       Temp:  27.50 degC   FIFO Time: 41718 us
[I] SI       67012 us   Accel:   -0.02     0.01     1.01 g   Gyro:       -        -        -       Temp:  27.00 degC   FIFO Time: 61718 us
[I] SI       86896 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 16182 us
[I] SI      106779 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20    -0.06     0.06 dps   Temp:  27.50 degC   FIFO Time: 36182 us
[I] SI      126663 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.12 dps   Temp:  27.50 degC   FIFO Time: 56182 us
[I] SI      146547 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20    -0.06     0.12 dps   Temp:  27.50 degC   FIFO Time: 10646 us
[I] SI      166430 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 30646 us
[I] SI      186314 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.00 degC   FIFO Time: 50646 us
[I] SI      206198 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time:  5110 us
[I] SI      226081 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.12 dps   Temp:  27.50 degC   FIFO Time: 25110 us
[I] SI      245965 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 45110 us
[I] SI      265849 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 65110 us
[I] SI      285733 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.06 dps   Temp:  27.50 degC   FIFO Time: 19574 us
[I] SI      305616 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20    -0.06     0.12 dps   Temp:  27.50 degC   FIFO Time: 39574 us
[I] SI      325500 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 59574 us
[I] SI      345384 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.00 degC   FIFO Time: 14038 us
[I] SI      365267 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.20     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 34038 us
[I] SI      385151 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time: 54038 us
[I] SI      405035 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26     0.00     0.12 dps   Temp:  27.50 degC   FIFO Time:  8502 us
[I] SI      424918 us   Accel:   -0.02     0.01     1.01 g   Gyro:   -2.26    -0.06     0.12 dps   Temp:  27.50 degC   FIFO Time: 28502 us
[I] Disabling SI print.
[I] Enabling LSB print.
[I] LSB    2751309 us   Accel:    -149       51     8308     Gyro:     -36        0        2       Temp:      4        FIFO Time:  9206 us
[I] LSB    2771193 us   Accel:    -150       55     8310     Gyro:     -36        2        2       Temp:      4        FIFO Time: 29206 us
[I] LSB    2791077 us   Accel:    -147       56     8314     Gyro:     -37       -1        2       Temp:      5        FIFO Time: 49206 us
[I] LSB    2810960 us   Accel:    -155       57     8308     Gyro:     -37       -3        2       Temp:      5        FIFO Time:  3670 us
[I] LSB    2830844 us   Accel:    -150       55     8304     Gyro:     -37        0        1       Temp:      5        FIFO Time: 23670 us
[I] LSB    2850728 us   Accel:    -143       51     8304     Gyro:     -37        1        1       Temp:      5        FIFO Time: 43670 us
[I] LSB    2870611 us   Accel:    -149       49     8305     Gyro:     -37        0        2       Temp:      5        FIFO Time: 63670 us
[I] LSB    2890495 us   Accel:    -150       50     8308     Gyro:     -37       -2        2       Temp:      4        FIFO Time: 18134 us
[I] LSB    2910379 us   Accel:    -155       55     8306     Gyro:     -37       -1        2       Temp:      5        FIFO Time: 38134 us
[I] LSB    2930263 us   Accel:    -155       58     8307     Gyro:     -37        0        2       Temp:      5        FIFO Time: 58134 us
[I] LSB    2950146 us   Accel:    -152       55     8306     Gyro:     -37        0        1       Temp:      5        FIFO Time: 12598 us
[I] LSB    2970030 us   Accel:    -153       52     8307     Gyro:     -36        1        1       Temp:      5        FIFO Time: 32598 us
[I] LSB    2989914 us   Accel:    -149       51     8309     Gyro:     -36        0        2       Temp:      5        FIFO Time: 52598 us
[I] LSB    3009797 us   Accel:    -145       51     8310     Gyro:     -37       -1        2       Temp:      5        FIFO Time:  7062 us
[I] LSB    3029681 us   Accel:    -153       51     8310     Gyro:     -37        0        2       Temp:      5        FIFO Time: 27062 us
[I] LSB    3049565 us   Accel:    -155       51     8310     Gyro:     -37        1        2       Temp:      5        FIFO Time: 47062 us
[I] LSB    3069448 us   Accel:    -150       53     8309     Gyro:     -37        0        2       Temp:      5        FIFO Time:  1526 us
```

