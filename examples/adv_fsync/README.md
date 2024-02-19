# adv_fsync

This application demonstrates how to enable EIS and get tagged gyro data from a FSYNC signal. This signal usually comes from a camera module that will be emulated here for the example. Here, the FSYNC signal is a 30Hz square signal and is connected to the IMU through the FSYNC pin.

It enables gyro in Low-Noise at 200Hz and emulates a camera module FSYNC signal at 30Hz to IMU FSYNC pin, being INT2 reconfigured for FSYNC usage only.
Upon INT1 trigger, gyro data and FSYNC tag can be either read from FIFO or from IMU Sensor registers, depending on user request through UART.

Gyro data will be reported with a FSYNC flag being set for the event happening next to the FSYNC rising edge (otherwise, the flag will be 0). The FSYNC delay counter represents the time between the FSYNC rising edge and the sampling.

It enables and configures FIFO to get FSYNC information by default.

## Command interface

This application allows the following command to be sent through UART:
* `f`: to get FSYNC flag and FSYNC counter from FIFO (this is the default at startup).
* `r`: to get FSYNC flag and FSYNC counter from IMU sensor registers.
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:
```
<timestamp> us Gyro: <gyr_x> <gyr_y> <gyr_z> dps [FSYNC event <fsync_count>]
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `<gyr_x|y|z>`: Raw gyroscope value converted in dps.
* `<fsync_count>`: Only if FSYNC signal was detected, time in us between the FSYNC rising edge and the current sampling time.

### Example of output

```
[I] ###
[I] ### Example FSYNC EIS (using advanced API)
[I] ###
[I]      75203 us   Gyro:   -2.01     0.31     0.06 dps
[I]      80166 us   Gyro:   -2.01     0.31     0.06 dps
[I]      85129 us   Gyro:   -1.95     0.24     0.06 dps   FSYNC event 1763
[I]      90091 us   Gyro:   -1.95     0.24     0.06 dps
[I]      95054 us   Gyro:   -1.95     0.18     0.06 dps
[I]     100017 us   Gyro:   -1.95     0.12     0.06 dps
[I]     104979 us   Gyro:   -1.95     0.06     0.00 dps
[I]     109942 us   Gyro:   -1.95     0.06     0.00 dps
[I]     114905 us   Gyro:   -1.95     0.06     0.06 dps
[I]     119867 us   Gyro:   -1.89     0.06     0.06 dps   FSYNC event 3181
[I]     124830 us   Gyro:   -1.95     0.12     0.06 dps
[I]     129793 us   Gyro:   -1.95     0.18     0.00 dps
[I]     134755 us   Gyro:   -1.95     0.18     0.06 dps
[I]     139718 us   Gyro:   -2.01     0.18     0.06 dps
[I]     144681 us   Gyro:   -2.01     0.24     0.06 dps
[I]     149643 us   Gyro:   -1.95     0.18     0.12 dps
[I]     154606 us   Gyro:   -1.95     0.18     0.06 dps   FSYNC event 4597
[I]     159569 us   Gyro:   -1.95     0.18     0.06 dps
[I]     164531 us   Gyro:   -1.95     0.18     0.06 dps
[I]     169494 us   Gyro:   -1.95     0.18     0.00 dps
[I]     174457 us   Gyro:   -1.95     0.18     0.00 dps
[I]     179419 us   Gyro:   -1.95     0.18     0.00 dps
[I]     184382 us   Gyro:   -2.01     0.24     0.06 dps   FSYNC event 1015
[I]     189345 us   Gyro:   -2.01     0.24     0.06 dps
[I]     194307 us   Gyro:   -2.01     0.24     0.12 dps
[I]     199270 us   Gyro:   -1.95     0.31     0.06 dps
[I]     204233 us   Gyro:   -1.95     0.24     0.06 dps
[I]     209195 us   Gyro:   -1.95     0.24     0.12 dps
[I]     214158 us   Gyro:   -1.95     0.18     0.06 dps
[I]     219121 us   Gyro:   -1.95     0.12     0.00 dps   FSYNC event 2432
```

We can verify the FSYNC signal frequency (expected to be 30 Hz) based on the last two FSYNC event detected above:

- fysnc_delta_time = (t1 - delay1) - (t0 - delay0)
- fsync_delta_time = (219121 - 2432) - (184382 - 1015)
- fysnc_delta_time = 33322 us = 29.999 Hz
