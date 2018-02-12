#include <cstdio>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <thread>
#include <mutex>
#include "GyroI2C.h"


using namespace std::chrono_literals;


enum reg {
    GYRO_ADDRESS = 0x6b,
};

int main(int argc, char *argv[]) {
    GyroI2C i2c(reg::GYRO_ADDRESS);
    i2c.calibrate();
    std::cout << std::fixed << std::setprecision(3);
    while (true) {
        std::cout << i2c.toString() << std::endl;
        std::this_thread::sleep_for(1s);
    }
}