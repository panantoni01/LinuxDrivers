#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<sys/ioctl.h>
#include"mydriver.h"

long check_status(int fd) {
    long status, ret = 0;

    ioctl(fd, MY_IOCTL_CHECK_STATUS, &status);

    if (status & STATUS_INVALID_OPERATION) {
        printf("Error: invalid operation\n");
        ret |= STATUS_INVALID_OPERATION;
    }
    if (status & STATUS_DIV_BY_ZERO) {
        printf("Error: div by zero\n");
        ret |= STATUS_DIV_BY_ZERO;
    }
    
    return ret;
}

void calculate(int fd, long num1, long num2, long op) {
    long result, error_mask;
    char op_char;
    
    write(fd, &num1, sizeof(num1));
    write(fd, &num2, sizeof(num2));
    ioctl(fd, MY_IOCTL_CHOOSE_OP, op);

    error_mask = check_status(fd);
    if (error_mask) 
        ioctl(fd, MY_IOCTL_RESET, error_mask);
    else {
        read(fd, &result, sizeof(result));
        if (op == ADD)
            op_char = '+';
        else if (op == SUB)
            op_char = '-';
        else if (op == MUL)
            op_char = '*';
        else if (op == DIV)
            op_char = '/';
        else
            op_char = '?';
        printf("%ld %c %ld = %ld\n", num1, op_char, num2, result);
    }
}

int main() {
    int fd;

    fd = open("/dev/chardev0", O_RDWR);

    calculate(fd,   15,   34, ADD);
    calculate(fd,    4,   34, MUL);
    calculate(fd,    2,    0, DIV);
    calculate(fd, 1234, 4321, SUB);
    calculate(fd,    6,    5, 100);

    close(fd);
    return 0;
}

