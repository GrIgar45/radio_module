#include <cstdio>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>


int main(int argc, char *argv[]) {
    wiringPiSetup();
    int fd = wiringPiI2CSetup(0x6b);
    if (fd == -1) {
        printf("Can't setup the I2C device\n");
        return -1;
    }
    int data = wiringPiI2CReadReg8(fd, 0x0f);
    if (data == 0xd4) {
        std::cout << "Success" << std::endl;
    } else {
        std::cout << std::hex << data << std::endl;
    }
    return 0;
}