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
    } else {
        for (;;) {
            static int sentCode = 0;
            std::cout << "Code: ";
            std::cin >> std::hex >> sentCode;
            if (!std::cin.good()) {
                break;
            }
            static int data = wiringPiI2CReadReg16(fd, sentCode);
            if (data == -1) {
                printf("No data\r");
            } else {
                printf("data=%d\r", data);
            }
        }
    }
    return 0;
}