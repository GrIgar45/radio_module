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

GyroI2C::GyroI2C(int deviceAddress)
        :
        axis_data {0, 0, 0}, noise_data {0, 0, 0}, last_data {0, 0, 0, 0, 0, 0} {
    wiringPiSetup();
    gyro = wiringPiI2CSetup(deviceAddress);
    if (gyro == -1) {
        throw std::runtime_error("Can't setup the I2C device.");
    }
    int data = wiringPiI2CReadReg8(gyro, ERegisters::WHO_AM_I);
    if (data != ERegisters::GYRO_NAME) {
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
    wiringPiI2CWriteReg8(gyro, ERegisters::CTRL_REG1, ERegisters::NORMAL_MODE);
    // Check the set value
    // It should be the same as it was established
    data = wiringPiI2CReadReg8(gyro, ERegisters::CTRL_REG1);
    if (data != ERegisters::NORMAL_MODE) {
        throw std::runtime_error("Can't turn on axis and switch to normal mode.");
    }
    /**
     * set FS = 2000 dps;
     * 0x00 - 250 dps;
     * 0x10 - 500 dps;
     * 0x20 - 2000 dps;
     */
    wiringPiI2CWriteReg8(gyro, ERegisters::CTRL_REG4, 0x20);
    data = wiringPiI2CReadReg8(gyro, ERegisters::CTRL_REG4);
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
    int tmp_axis_data[n];
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start) <
           (milliseconds - 2s)) {
        for (int i = 0; i < n; i++) {
            tmp_axis_data[i] = wiringPiI2CReadReg8(gyro, ERegisters::OUT_X_L + i);
        }
        for (int i = 0; i < 3; i++) {
            auto j = i << 1;
            auto d = std::abs(normalizationAxis(tmp_axis_data[j + 1], tmp_axis_data[j]));
            noise_data[i] = (d > noise_data[i] && d < 100) ? d : noise_data[i];
        }
        while ((wiringPiI2CReadReg8(gyro, ERegisters::IS_NEW_DATA_READY) & ERegisters::DATA_READY) != ERegisters::DATA_READY) {
            std::this_thread::sleep_for(1ms);
        }
    }
    for (int i = 0; i < 3; i++) {
        noise_data[i] = noise_data[i] * 2;
    }
#ifndef NDEBUG
    std::stringstream s;
    s << std::fixed << std::setprecision(3);
    s << "Calibration successful. X: " << noise_data[0] << " Y: " << noise_data[1] << " Z: " << noise_data[2]
      << std::endl;
    std::cout << s.str();
#endif
    calibrated = 1;
    run = true;
    reading = new std::thread(&GyroI2C::readData, this);
}

void GyroI2C::stop() {
    run = false;
    reading->join();
    delete reading;
}

std::ostream &operator<<(std::ostream &s, const GyroI2C &data) {
    s << "Position\n";
    s << std::setfill(' ');
    s << "X: " << std::setw(7) << data.getX();
    s << "\tY: " << std::setw(7) << data.getY();
    s << "\tZ: " << std::setw(7) << data.getZ();
    return s;
}

#ifndef NDEBUG
std::string GyroI2C::toStringLastData() {
    std::stringstream s;
    s << "Last raw\n";
    s << std::setfill(' ');
    s << "X: " << std::setw(3) << last_data[0] << ", " << std::setw(3) << last_data[1];
    s << "\tY: " << std::setw(3) << last_data[2] << ", " << std::setw(3) << last_data[3];
    s << "\tZ: " << std::setw(3) << last_data[4] << ", " << std::setw(3) << last_data[5];
    return s.str();
}
#endif

float GyroI2C::getX() const {
    return axis_data[0];
}

float GyroI2C::getY() const {
    return axis_data[1];
}

float GyroI2C::getZ() const {
    return axis_data[2];
}

void GyroI2C::readData() {
    const auto n = 6;
    int *delivered_data = last_data;
    while (run) {
        for (int i = 0; i < n; i++) {
            delivered_data[i] = wiringPiI2CReadReg8(gyro, ERegisters::OUT_X_L + i);
        }
        for (int i = 0; i < 3; i++) {
            auto j = i << 1;
            auto d = normalizationAxis(delivered_data[j + 1], delivered_data[j]);
            axis_data[i] += (std::abs(d) > noise_data[i]) ? d : 0.f;
        }
        while ((wiringPiI2CReadReg8(gyro, ERegisters::IS_NEW_DATA_READY) & ERegisters::DATA_READY) != ERegisters::DATA_READY) {
            std::this_thread::sleep_for(1ms);
        }
    }
}

float GyroI2C::normalizationAxis(int H, int L) {
    // is it H less than 128 ? the data is positive : negative
    auto sign = (H < 0x7f) ? 1 : -1;
    auto value = (H << 8) | L;
    if (sign == -1) {
        value = value - 0xff;
    }
    /**
     * Shift the high bits and remove the sign value.
     + FS * 0.001 * microsecond spend
     + FS = 250  dps     8.75 mdps/digit
     + FS = 500  dps     17.50
     + FS = 2000 dps     70
     */
    return value * 0.00070f;
}
