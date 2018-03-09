#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <mutex>
#include "I2CDevices/AccelerometerI2C.h"


using namespace std::chrono_literals;

enum reg {
    GYRO_ADDRESS = 0x6b,
    ACCELER_ADDRESS = 0x1d
};

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "Fock u. Use 3 digits after " << argv[0] << std::endl;
        return 1;
    }
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
    i2C.calibrate(atoi(argv[1]), atoi(argv[2]), atoi(argv[3]));
    while (true) {
        std::cout << i2C << std::endl;
        std::this_thread::sleep_for(100ms);
    }
}