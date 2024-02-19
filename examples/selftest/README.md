# selftest

This application demonstrates how to execute the Self-test.

Selftest procedure is run once every second. 

Selftest for accel and gyro can be enabled independently through UART link.
Accel and gyro status are printed on the terminal for each selftest procedure run.

**Warning:** The board needs to be static during the execution of the Self-Test example.

## Command interface

This application allows the following command to be sent through UART:
* `a`: to enable/disable selftest accel.
* `g`: to enable/disable selftest gyro.
* `c`: to print current configuration.
* `h`: to print help screen.

## Terminal output

```
[I] ###
[I] ### Example SELFTEST
[I] ###
[I] #
[I] # Current configuration
[I] #
[I] # Selftest accel Enabled.
[I] # Selftest gyro Enabled.
[I] #
[I] Running Selftest...
[I] Gyro Selftest PASS
[I] Accel Selftest PASS
[I] Running Selftest...
[I] Gyro Selftest PASS
[I] Accel Selftest PASS
[I] Running Selftest...
[I] Gyro Selftest PASS
[I] Accel Selftest PASS
```

