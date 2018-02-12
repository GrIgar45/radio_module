//
// Created by dev on 2/9/18.
//

#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <sstream>
#include <iomanip>
#include "GyroI2C.h"


using namespace std::chrono_literals;

GyroI2C::GyroI2C(int deviceAddress) {
    wiringPiSetup();
    gyro = wiringPiI2CSetup(deviceAddress);
    if (gyro == -1) {
        throw std::runtime_error("Can't setup the I2C device.");
    }
    int data = wiringPiI2CReadReg8(gyro, (int)reg::WHO_AM_I);
    if (data != (int)reg::GYRO_NAME) {
        throw std::runtime_error("L3GD20 is not working.");
    }
    std::this_thread::sleep_for(10ms);
    /**
     * Also here we switch to the normal mode and turn on all three axis
     * 0x0F
     * 0000  1111
     * power ^|||
     * z axis ^||
     * y axis  ^|
     * x axis   ^
     */
    wiringPiI2CWriteReg8(gyro, (int)reg::CTRL_REG1, (int)reg::NORMAL_MODE);
    // Check the set value
    // It should be the same as it was established
    data = wiringPiI2CReadReg8(gyro, (int)reg::CTRL_REG1);
    if (data != (int)reg::NORMAL_MODE) {
        throw std::runtime_error("Can't turn on axis and switch to normal mode.");
    }
    /**
     * set FS = 2000 dps;
     * 0x00 - 250 dps;
     * 0x10 - 500 dps;
     * 0x20 - 2000 dps;
     */
    wiringPiI2CWriteReg8(gyro, (int)reg::CTRL_REG4, 0x20);
    data = wiringPiI2CReadReg8(gyro, (int)reg::CTRL_REG4);
    if (data != 0x20) {
        throw std::runtime_error("Can't set DPS value.");
    }
}

void GyroI2C::calibrate() {
    this->calibrate(1000ms);
}

void GyroI2C::calibrate(std::chrono::milliseconds milliseconds) {
    for (auto noise : noiseData) {
        noise = .0;
    }
    const auto n = 6;
    int dData[n];
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start) <
           milliseconds) {
        for (int i = 0; i < n; i++) {
            dData[i] = wiringPiI2CReadReg8(gyro, (int)reg::OUT_X_L + i);
        }
        for (int i = 0; i < 3; i++) {
            auto j = i << 1;
            auto d = normalizationAxis(dData[j + 1], dData[j]);
            noiseData[i] = (d > noiseData[i]) ? d : noiseData[i];
        }
    }
    calibrated = 1;
    reading = new std::thread(&GyroI2C::readData, this);
}

void GyroI2C::stop() {
    run = false;
    reading->join();
    delete reading;
}

std::string GyroI2C::toString() {
    std::stringstream s;
    s << std::fixed << std::setprecision(3);
    s << "X: " << getX();
    s << "\tY: " << getY();
    s << "\tZ: " << getZ();
    return s.str();
}

double inline GyroI2C::getX() {
    return axisData[0];
}

double inline GyroI2C::getY() {
    return axisData[1];
}

double inline GyroI2C::getZ() {
    return axisData[2];
}

void GyroI2C::readData() {
    const auto n = 6;
    int deliveredData[n];
    while (run) {
        for (int i = 0; i < n; i++) {
            deliveredData[i] = wiringPiI2CReadReg8(gyro, 0x28 + i);
        }
        affordable.lock();
        for (int i = 0; i < 3; i++) {
            auto j = i << 1;
            auto data = normalizationAxis(deliveredData[j + 1], deliveredData[j]);
            axisData[i] += (data > noiseData[i]) ? data : 0;
        }
        affordable.unlock();
        while ((wiringPiI2CReadReg8(gyro, 0x27) & 0x8) != 0x8) {
            std::this_thread::sleep_for(2ms);
        }
    }

}

double GyroI2C::normalizationAxis(int H, int L) {
    /**
     * Shift the high bits and remove the sign value.
     + FS * 0.001 * microsecond spend
     + FS = 250  dps     8.75 mdps/digit
     + FS = 500  dps     17.50
     + FS = 2000 dps     70
     */
    auto sign = ((H & 0x80) == 0) ? 1 : -1;
    return ((H << 8 | L) & 0x7fff) * 0.07 * sign;

}
