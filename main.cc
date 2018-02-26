#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <mutex>
#include "I2CDevices/GyroI2C.h"
#include "I2CDevices/AccelerometerI2C.h"


using namespace std::chrono_literals;

enum reg {
    GYRO_ADDRESS = 0x6b,
    ACCELER_ADDRESS = 0x1d
};

int main(int argc, char *argv[]) {
//    GyroI2C i2c(reg::GYRO_ADDRESS);
//    i2c.calibrate();
//    while (true) {
//#ifndef NDEBUG
//        std::cout << i2c.toStringLastData() << std::endl;
//#endif
//        std::cout << i2c << std::endl;
//        std::this_thread::sleep_for(100ms);
//    }
    AccelerometerI2C i2C(reg::ACCELER_ADDRESS);
    while (true) {
        i2C.calibrate();
        std::cout << i2C << std::endl;
    }
}