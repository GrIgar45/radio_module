#include <cstdio>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>


namespace add {
//    std::ofstream log("log.txt", std::ofstream::trunc);
    const int DELAY = 20;
}

enum reg {
    GYRO_ADRESS = 0x6b,
    WHO_AM_I = 0x0f,
    GYRO_NAME = 0xd4,
    CTRL_REG1 = 0x20,
    NORMAL_MODE = 0x0f
};

void readData(int &fd, float *outData) {
    std::chrono::high_resolution_clock::time_point newMeasuring, lastMeasuring;
    lastMeasuring = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::duration timeSpend {};
    while (true) {
        static const int n = 6;
        static int deliveredData[n];
        lastMeasuring = newMeasuring;
        newMeasuring = std::chrono::system_clock::now();
        for (int i = 0; i < n; i++) {
            deliveredData[i] = wiringPiI2CReadReg8(fd, 0x28 + i);
        }
        for (int i = 0; i < 3; i++) {
            static int j = i << 1;
            // If the highest bit is high, the sign is negative.
            static int sign = (deliveredData[j + 1] & 0x8000) ? -1 : 1;
            // Shift the high bits and remove the sign value.
            //
            timeSpend = newMeasuring - lastMeasuring;
            outData[i] += ((deliveredData[j + 1] << 8 | deliveredData[j]) & 0x7fff * sign)
                          * 0.07f * (std::chrono::duration_cast<std::chrono::microseconds>(timeSpend).count() * 0.001f);
            if (outData[i] < 0x0a) {
                outData[i] = 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(add::DELAY));;
    }
}

int main(int argc, char *argv[]) {
    wiringPiSetup();
    std::cout << "Start initialize L3GD20" << std::endl;
    int fd = wiringPiI2CSetup(reg::GYRO_ADRESS);
    if (fd == -1) {
        std::cerr << "Can't setup the I2C device" << std::endl;
        return 1;
    }
    std::cout << "Ok. Check reading." << std::endl;
    {
        int data = wiringPiI2CReadReg8(fd, reg::WHO_AM_I);
        if (data != reg::GYRO_NAME) {
            std::cerr << "L3GD20 is not working." << std::endl;
            return 1;
        }
        delay(10);
        std::cout << "Ok. Read is finished successfully. Now check writing" << std::endl;

        /**
         * Also here we switch to the normal mode and turn on all three axis
         * 0x0F
         * 0000  1111
         * power ^|||
         * z axis ^||
         * y axis  ^|
         * x axis   ^
         */
        wiringPiI2CWriteReg8(fd, reg::CTRL_REG1, reg::NORMAL_MODE);
        // Check the set value
        // It should be the same as it was established
        data = wiringPiI2CReadReg8(fd, reg::CTRL_REG1);
        if (data != reg::NORMAL_MODE) {
            std::cerr << "Writing isn't worked" << std::endl;
        }
        wiringPiI2CWriteReg8(fd, 0x23, 0x30);
    }
    std::cout << "Getting data" << std::endl;
    float data[] = {.0, .0, .0};
    std::cout << std::fixed << std::setprecision(3);
    std::thread getData(readData, std::ref(fd), std::ref(data));
    while (true) {
        for (float d : data) {
            std::cout << d << ": ";
        }
        std::cout << std::endl;
        delay(1000);
    }
    return 0;
}