#include <cstdio>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>


int main(int argc, char *argv[]) {
    wiringPiSetup();
    std::cout << "Start initialize L3GD20" << std::endl;
    int fd = wiringPiI2CSetup(0x6b);
    if (fd == -1) {
        printf("Can't setup the I2C device\n");
        return -1;
    }
    std::cout << "Ok. Check reading" << std::endl;
    int data = wiringPiI2CReadReg8(fd, 0x0f);
    if (data != 0xd4) {
        std::cout << "L3GD20 is not working" << std::endl;
        return 1;
    }
    delay(10);
    std::cout << "Ok. Read is finished successfully. Now check writing" << std::endl;
    wiringPiI2CWriteReg8(fd, 0x20, 0x0f);
    data = wiringPiI2CReadReg8(fd, 0x20);
    std::cout << std::hex << data << std::endl;
    return 0;
}