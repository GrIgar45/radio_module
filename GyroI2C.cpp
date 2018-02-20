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
        axisData {0, 0, 0}, noiseData {0, 0, 0}, lastData {0, 0, 0, 0, 0, 0} {
    wiringPiSetup();
    gyro = wiringPiI2CSetup(deviceAddress);
    if (gyro == -1) {
        throw std::runtime_error("Can't setup the I2C device.");
    }
    int data = wiringPiI2CReadReg8(gyro, reg::WHO_AM_I);
    if (data != reg::GYRO_NAME) {
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
    wiringPiI2CWriteReg8(gyro, reg::CTRL_REG1, reg::NORMAL_MODE);
    // Check the set value
    // It should be the same as it was established
    data = wiringPiI2CReadReg8(gyro, reg::CTRL_REG1);
    if (data != reg::NORMAL_MODE) {
        throw std::runtime_error("Can't turn on axis and switch to normal mode.");
    }
    /**
     * set FS = 2000 dps;
     * 0x00 - 250 dps;
     * 0x10 - 500 dps;
     * 0x20 - 2000 dps;
     */
    wiringPiI2CWriteReg8(gyro, reg::CTRL_REG4, 0x20);
    data = wiringPiI2CReadReg8(gyro, reg::CTRL_REG4);
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
            dData[i] = wiringPiI2CReadReg8(gyro, reg::OUT_X_L + i);
        }
        for (int i = 0; i < 3; i++) {
            auto j = i << 1;
            auto d = std::abs(normalizationAxis(dData[j + 1], dData[j]));
            noiseData[i] = (d > noiseData[i] && d < 100) ? d : noiseData[i];
        }
        while ((wiringPiI2CReadReg8(gyro, 0x27) & 0x8) != 0x8) {
            std::this_thread::sleep_for(1ms);
        }
    }
    for (int i = 0; i < 3; i++) {
        noiseData[i] = noiseData[i] * 2;
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

std::ostream &operator<<(std::ostream &s, const GyroI2C &data) {
    s << "Position\n";
    s << std::setfill(' ');
    s << "X: " << std::setw(7) << data.getX();
    s << "\tY: " << std::setw(7) << data.getY();
    s << "\tZ: " << std::setw(7) << data.getZ();
    return s;
}

std::string GyroI2C::toStringLastData() {
    std::stringstream s;
    s << "Last raw\n";
    s << std::setfill(' ');
    s << "X: " << std::setw(3) << lastData[0] << ", " << std::setw(3) << lastData[1];
    s << "\tY: " << std::setw(3) << lastData[2] << ", " << std::setw(3) << lastData[3];
    s << "\tZ: " << std::setw(3) << lastData[4] << ", " << std::setw(3) << lastData[5];
    return s.str();
}

int GyroI2C::getX() const {
    return axisData[0];
}

int GyroI2C::getY() const {
    return axisData[1];
}

int GyroI2C::getZ() const {
    return axisData[2];
}

void GyroI2C::readData() {
    const auto n = 6;
    int *deliveredData = lastData;
    while (run) {
        for (int i = 0; i < n; i++) {
            deliveredData[i] = wiringPiI2CReadReg8(gyro, reg::OUT_X_L + i);
        }
        for (int i = 0; i < 3; i++) {
            auto j = i << 1;
            auto d = normalizationAxis(deliveredData[j + 1], deliveredData[j]);
            axisData[i] += (std::abs(d) > noiseData[i]) ? static_cast<int>(d) : 0;
        }
        while ((wiringPiI2CReadReg8(gyro, reg::IS_NEW_DATA_READY) & reg::DATA_READY) != reg::DATA_READY) {
            std::this_thread::sleep_for(1ms);
        }
    }
}

float GyroI2C::normalizationAxis(int H, int L) {
    // is it H less than 128 ? the data is positive : negative
    auto sign = (H < 0x7f) ? 1 : -1;
    if (sign == -1) {
        H = 0xff - H;
    }
    /**
     * Shift the high bits and remove the sign value.
     + FS * 0.001 * microsecond spend
     + FS = 250  dps     8.75 mdps/digit
     + FS = 500  dps     17.50
     + FS = 2000 dps     70
     */
    return (H << 8 | L) * 0.0070f * sign;
}
