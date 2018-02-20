#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <mutex>
#include "GyroI2C.h"
#include "AccelerometerI2C.h"


using namespace std::chrono_literals;

enum reg {
    GYRO_ADDRESS = 0x6b,
    ACCELER_ADDRESS = 0x1d
};

int main(int argc, char *argv[]) {
//    GyroI2C i2c(reg::GYRO_ADDRESS);
//    i2c.calibrate();
//    while (true) {
//        std::cout << i2c.toStringLastData() << std::endl;
//        std::cout << i2c << std::endl;
//        std::this_thread::sleep_for(100ms);
//    }
    AccelerometerI2C i2C(reg::ACCELER_ADDRESS);
}