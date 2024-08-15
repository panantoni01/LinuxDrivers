# `driver_iio_si7020` description

The driver controls a SI7021 device, which is a temperature and humidity sensor. The datasheet is available on [Silabs site](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf).

This IIO driver has already existed in the Linux kernel. With the help of this repo, the support for manipulating the integrated on-chip heater was implemented. The patch that was sent and accepted to the Linux kernel can be viewed [here](https://github.com/torvalds/linux/commit/2aac3f9aec74b28ea73ad96efbcc0c56e5ff814f)
