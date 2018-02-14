//
// Created by dev on 2/9/18.
//

#include <cmath>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "GyroI2C.h"


using namespace std::chrono_literals;

GyroI2C::GyroI2C(int deviceAddress, int axisData) : axisData {0, 0, 0}, noiseData {0, 0, 0} {
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
    this->calibrate(5000ms);
}

void GyroI2C::calibrate(std::chrono::milliseconds milliseconds) {
    std::this_thread::sleep_for(2s);
    const auto n = 6;
    int dData[n];
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start) <
           (milliseconds - 2s)) {
        for (int i = 0; i < n; i++) {
            dData[i] = wiringPiI2CReadReg8(gyro, (int)reg::OUT_X_L + i);
        }
        for (int i = 0; i < 3; i++) {
            auto j = i << 1;
            auto d = std::abs(normalizationAxis(dData[j + 1], dData[j]));
            noiseData[i] = (d > noiseData[i] && d < 100) ? d : noiseData[i];
        }
    }
    for (auto noise : noiseData) {
        noise = noise * 10;
    }
    std::stringstream s;
    s << std::fixed << std::setprecision(3);
    s << "Calibration successful. X: " << noiseData[0] << " Y: " << noiseData[1] << " Z: " << noiseData[2] << std::endl;
    std::cout << s.str();
    calibrated = 1;
    run = true;
    reading = new std::thread(&GyroI2C::readData, this);
}

void GyroI2C::stop() {
    run = false;
    reading->join();
    delete reading;
}

std::string GyroI2C::toString() {
    std::stringstream s;
    s << "Position\n";
    s << "X: " << getX();
    s << "\tY: " << getY();
    s << "\tZ: " << getZ();
    return s.str();
}

std::string GyroI2C::toStringLastData() {
    std::stringstream s;
//    s << std::fixed << std::setprecision(3);
    s << "X: " << lastData[0];
    s << "\tY: " << lastData[1];
    s << "\tZ: " << lastData[2];
    return s.str();
}

int inline GyroI2C::getX() {
    affordable.lock();
    return axisData[0];
    affordable.unlock();
}

int inline GyroI2C::getY() {
    affordable.lock();
    return axisData[1];
    affordable.unlock();
}

int inline GyroI2C::getZ() {
    affordable.lock();
    return axisData[2];
    affordable.unlock();
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
            lastData[i] = normalizationAxis(deliveredData[j + 1], deliveredData[j]);
            axisData[i] += (std::abs(lastData[i]) > noiseData[i]) ? static_cast<int>(lastData[i]) : 0;
        }
        affordable.unlock();
        while ((wiringPiI2CReadReg8(gyro, 0x27) & 0x8) != 0x8) {
            std::this_thread::sleep_for(2ms);
        }
    }
}

float GyroI2C::normalizationAxis(int H, int L) {
    /**
     * Shift the high bits and remove the sign value.
     + FS * 0.001 * microsecond spend
     + FS = 250  dps     8.75 mdps/digit
     + FS = 500  dps     17.50
     + FS = 2000 dps     70
     */
    auto sign = ((H & 0x80) == 0) ? 1 : -1;
    return ((H << 8 | L) & 0x7fff) * 0.07f * sign;
}
