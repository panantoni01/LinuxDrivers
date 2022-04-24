#include<stdio.h>
#include<fcntl.h>
#include<unistd.h>
#include<sys/ioctl.h>
#include"my_driver.h"

int main() {
    int fd;
    long var, num_to_send;

    fd = open("/dev/chdev0", O_RDWR);

    read(fd, &var, sizeof(long));
    printf("Var initially: %ld\n", var);

    num_to_send = 15;
    write(fd, &num_to_send, sizeof(long));
    num_to_send = 34;
    write(fd, &num_to_send, sizeof(long));

    read(fd, &var, sizeof(long));
    printf("15 + 34 = %ld\n", var);

    ioctl(fd, MY_IOCTL_CHANGE_OP, MUL);

    num_to_send = 49;
    write(fd, &num_to_send, sizeof(long));

    read(fd, &var, sizeof(long));
    printf("49 * 49 = %ld\n", var);
    
    close(fd);
    return 0;
}

