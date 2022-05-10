#pragma once
#include <asm/ioctl.h>

#define ADD (1 << 0)
#define SUB (1 << 1)
#define MUL (1 << 2)
#define DIV (1 << 3)

#define STATUS_INVALID_OPERATION (1 << 0)
#define STATUS_DIV_BY_ZERO (1 << 1)

#define MY_IOCTL_RESET _IOW('M', 0, long)
#define MY_IOCTL_CHOOSE_OP _IOW('M', 1, long)
#define MY_IOCTL_CHECK_STATUS _IOR('M', 2, long)