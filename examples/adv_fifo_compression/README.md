# adv_fifo_compression

This application demonstrates how to retrieve data from the FIFO while using FIFO compression.

It enables accel and/or gyro in Low-Noise mode at 50 Hz and enables/disable compression upon user request through UART.

FIFO compression is enabled by default, with an interrupt threshold of 16 packets and a non-compressed frame every 8 frames.

## Command interface

This application allows the following command to be sent through UART:
* `a`: to enable/disable accelerometer (defaults to enable).
* `g`: to enable/disable gyroscope (defaults to enable).
* `f`: to enable/disable FIFO compression (defaults to disable).
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:

```
SI <timestamp> us FIFO count <fifo_count> Sample count <sample_count>
SI <timestamp> us Average Accel: <acc_x> <acc_y> <acc_z> g Average Gyro: <gyr_x> <gyr_y> <gyr_z> dps Average Temp: <temp> degC
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `<fifo_count>`: Packet count read from FIFO, should be always 16 considering FIFO watermark configuration.
* `<sample_count>`: Total count of samples once uncompression of `<fifo_count>` packets is done, will be 16 if FIFO compression is disabled.
* `<acc_x|y|z>`: Average raw accel value of all `<sample_count>` samples converted in g.
* `<gyr_x|y|z>`: Average raw gyroscope value of all `<sample_count>` samples converted in dps.
* `<temp>`: Average raw temp value of all `<sample_count>` samples converted in °C

### Example of output

```
[I] ###
[I] ### Example FIFO Compression (using advanced API)
[I] ###
[I] SI     444817 us   FIFO count 16   Sample count 23
[I] SI     444817 us   Average Accel:   -0.04     0.01     1.01 g   Average Gyro:   -0.86     5.72     0.21 dps   Average Temp:  24.37 degC
[I] SI    1637743 us   FIFO count 16   Sample count 60
[I] SI    1637743 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.97     0.03     0.08 dps   Average Temp:  24.47 degC
[I] SI    2592083 us   FIFO count 16   Sample count 48
[I] SI    2592083 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.98     0.01     0.09 dps   Average Temp:  24.37 degC
[I] SI    3745245 us   FIFO count 16   Sample count 58
[I] SI    3745245 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.97     0.01     0.08 dps   Average Temp:  24.44 degC
[I] SI    4898406 us   FIFO count 16   Sample count 58
[I] SI    4898406 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.98     0.01     0.07 dps   Average Temp:  24.41 degC
[I] SI    5972039 us   FIFO count 16   Sample count 54
[I] SI    5972039 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.97     0.02     0.07 dps   Average Temp:  24.41 degC
[I] SI    7125200 us   FIFO count 16   Sample count 58
[I] SI    7125200 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.98     0.01     0.07 dps   Average Temp:  24.43 degC
[I] SI    8198833 us   FIFO count 16   Sample count 54
[I] SI    8198833 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.97     0.01     0.08 dps   Average Temp:  24.42 degC
[I] SI    9351994 us   FIFO count 16   Sample count 58
[I] SI    9351994 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.97     0.01     0.07 dps   Average Temp:  24.41 degC
[I] SI   10505156 us   FIFO count 16   Sample count 58
[I] SI   10505156 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.98     0.01     0.07 dps   Average Temp:  24.46 degC
[I] SI   11717964 us   FIFO count 16   Sample count 61
[I] SI   11717964 us   Average Accel:   -0.05     0.00     1.01 g   Average Gyro:   -0.98     0.01     0.07 dps   Average Temp:  24.45 degC

```

