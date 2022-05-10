# Lab 3
The driver controls a simple arithmetic peripheral, that supports +,-,*,/ operations. It is simulated by in a virtual environment using Renode. The driver provides the following functionalities:

* IOCTL for selecting the operation
* IOCTL for checking status of the device
* IOCTL for resetting error code
* data is provided to the driver by writing the /dev file
* results are fetched by reading the /dev file


Example flow:

```
write <-- 2
write <-- 10
ioctl operation_add
ioctl check_status
read -> 12
```

Example error handling flow:

```
write <-- 2
write <-- 0
ioctl operation_div
ioctl check_status
ioctl error_ack
print error
```

* ```my_driver.c``` - main driver code
* ```my_driver.h``` - separate header file with defines for ioctls
* ```mydriver_test.c``` -  example userspace program to test the driver functionality
* ```rv32.dts``` - device tree file - contains hardware description (including the peripheral).