# adv_wom

This application demonstrates how to collect WOM interrupt.

Once the example is running, moving the board will generate WOM events. It starts accel in LowPower at 12.5Hz and will notify a WOM through INT1 when any of the 3 axis exceeds the threshold. The axis on which the WOM has been detected will then be printed on the terminal.

The default configuration will lead to WOM event being generated when a 200 mg acceleration is measured. The threshold can be configured through UART to be either 200 mg (so called *High* threshold) or 24mg (so called *Low* threshold).

## Command interface

This application allows the following command to be sent through UART:
* `w`: to toggle low/high WOM threshold (defaults to high).
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

### Data format

Data are printed on the terminal as follow:
```
WoM at    <timestamp> (X, Y, Z): <x>, <y>, <z>
```

With:
* `<timestamp>`: Time in microsecond read from MCU clock when latest INT1 was fired.
* `<x>`: 1 when motion through X axis exceeds configured threshold, 0 otherwise.
* `<y>`: 1 when motion through Y axis exceeds configured threshold, 0 otherwise.
* `<z>`: 1 when motion through Z axis exceeds configured threshold, 0 otherwise.

### Example of output

```
[I] ###
[I] ### Example WOM
[I] ###
[I] #
[I] # Current configuration
[I] #
[I] # WOM threshold: High
[I] #
[I] WoM at    3367001 (X, Y, Z): 0, 1, 0
[I] WoM at    3446707 (X, Y, Z): 1, 1, 0
[I] WoM at    3526415 (X, Y, Z): 0, 1, 0
[I] WoM at    3606128 (X, Y, Z): 0, 1, 0
[I] WoM at    3765558 (X, Y, Z): 0, 1, 0
[I] WoM at    4084384 (X, Y, Z): 0, 1, 0
[I] WoM at    4164086 (X, Y, Z): 0, 1, 0
```

