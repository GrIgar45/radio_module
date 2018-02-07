#include <cstdio>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>

void readData(int &fd, float &outData[]) {
    static const int n = 6;
    static int deliveredData[n];
    for (int i = 0; i < n; i++) {
        deliveredData[i] = wiringPiI2CReadReg8(fd, 0x28 + i);
    }
    for (int i = 0; i < 3; i++) {
        static int j = i * 2;
        outData[i] = (deliveredData[j + 1] << 16 | deliveredData[j]) * 0.00875f;
    }
}

int main(int argc, char *argv[]) {
    wiringPiSetup();
    std::cout << "Start initialize L3GD20" << std::endl;
    int fd = wiringPiI2CSetup(0x6b);
    if (fd == -1) {
        std::cerr << "Can't setup the I2C device" << std::endl;
        return -1;
    }
    std::cout << "Ok. Check reading." << std::endl;
    {
        int data = wiringPiI2CReadReg8(fd, 0x0f);
        if (data != 0xd4) {
            std::cerr << "L3GD20 is not working." << std::endl;
            return 1;
        }
        delay(10);
        std::cout << "Ok. Read is finished successfully. Now check writing" << std::endl;
        wiringPiI2CWriteReg8(fd, 0x20, 0x0f);
        data = wiringPiI2CReadReg8(fd, 0x20);
        if (data != 0x0f) {
            std::cerr << "Writing isn't worked" << std::endl;
        }
    }
    float data[3];
    while (true) {
        readData(fd, data);
        std::cout << "                                  ";
        for (float d : data) {
            std::cout << d;
        }
        std::cout << "\r" << std::endl;
    }
    return 0;
}