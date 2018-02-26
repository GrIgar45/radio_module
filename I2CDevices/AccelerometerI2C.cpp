//
// Created by dev on 2/19/18.
//

#include <cmath>
#include <iomanip>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "AccelerometerI2C.h"


using namespace std::chrono_literals;

AccelerometerI2C::AccelerometerI2C(int deviceAddress) : axis_data { 0, 0, 0 }, last_data { 0, 0, 0 } {
    wiringPiSetup();
    accelerometer_file_description = wiringPiI2CSetup(deviceAddress);
    if (accelerometer_file_description == -1) {
        throw std::runtime_error("Can't setup the I2C device.");
    }
    for (int i = 0; i < 10; i++) {
        int data = wiringPiI2CReadReg8(accelerometer_file_description, (int)ERegisters::WHAT_IS_MY_ADDRESS);
        std::cout << std::hex << data << std::endl;
    }
    this->setSensitivityAndMode(ESensitivity::G2, EMode::MEASURE);
}

void AccelerometerI2C::calibrate() {
    calibrate(5s);
}

void AccelerometerI2C::calibrate(std::chrono::milliseconds milliseconds) {
    std::this_thread::sleep_for(2s);
    milliseconds = milliseconds - 2s;
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast < std::chrono::milliseconds >(std::chrono::steady_clock::now() - start) <
           milliseconds) {
        for (int i = 0; i < 3; i++) {
            auto d = std::abs(last_data[i]);
            noise_data[i] = (d > noise_data[i] && d < 100) ? d : noise_data[i];
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
    reading = new std::thread(&AccelerometerI2C::reading, this);
}

bool AccelerometerI2C::setSensitivity(AccelerometerI2C::ESensitivity sensitivity) {
    int selected;
    switch (sensitivity) {
        case ESensitivity::G2:
            this->sensitivity = 2;
            selected = ERegisters::MCTL_2G;
            break;
        case ESensitivity::G4:
            this->sensitivity = 4;
            selected = ERegisters::MCTL_4G;
            break;
        case ESensitivity::G8:
            this->sensitivity = 8;
            selected = ERegisters::MCTL_8G;
            break;
    }
    return this->writeMctlValueAndCheck(selected, ERegisters::MCTL_GLVL_MASK);
}

bool AccelerometerI2C::setMode(AccelerometerI2C::EMode mode) {
    int selected;
    switch (mode) {
        case EMode::STANDBY:
            selected = ERegisters::MCTL_STANDBY;
            break;
        case EMode::MEASURE:
            selected = ERegisters::MCTL_MEASUMENT;
            break;
        case EMode::LEVEL:
            selected = ERegisters::MCTL_LEVEL;
            break;
        case EMode::PULSE:
            selected = ERegisters::MCTL_PULSE;
            break;
    }
    return this->writeMctlValueAndCheck(selected, ERegisters::MCTL_MODE_MASK);
}

bool AccelerometerI2C::setSensitivityAndMode(AccelerometerI2C::ESensitivity sensitivity, AccelerometerI2C::EMode mode) {
    return this->setSensitivity(sensitivity) && this->setMode(mode);
}

void AccelerometerI2C::stop() {

}

std::ostream &operator<<(std::ostream &ostream, const AccelerometerI2C &data) {
    ostream << "Position\n";
    ostream << std::setfill(' ');
    ostream << "X: " << std::setw(7) << data.getX();
    ostream << "\tY: " << std::setw(7) << data.getY();
    ostream << "\tZ: " << std::setw(7) << data.getZ();
    return ostream;
}

//std::string AccelerometerI2C::toStringLastData() {
//    return std::__cxx11::string();
//}

float AccelerometerI2C::getX() const {
    return this->axis_data[0];
}

float AccelerometerI2C::getY() const {
    return this->axis_data[1];
}

float AccelerometerI2C::getZ() const {
    return this->axis_data[2];
}

void AccelerometerI2C::setAxisOffset(int x, int y, int z) {

}

void AccelerometerI2C::readingLoop() {
    while (run) {
        read10BitData(last_data);
    }
}

float AccelerometerI2C::normalizationAxisToGValue(int high_byte, int low_byte) {
    high_byte &= ERegisters::HIGH_BIT_MASK;
    low_byte &= ERegisters::LOW_BIT_MASK;
    int i = (high_byte << 8) | low_byte;

    // todo negative value


    float value = 2.0f * sensitivity;
    value /= 1024.f;
    value *= i;
    return value;
}

void AccelerometerI2C::read10BitData(float *XYZ) {
    const int n = 3;
    for (int i = 0; i < n; ++i) {
        auto l = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_10_BIT + i * 2);
        auto h = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_10_BIT + i * 2 + 1);
        XYZ[i] = normalizationAxisToGValue(h, l);
    }
}

float AccelerometerI2C::read10BitData(AccelerometerI2C::EAxis axis) {
    int l, h;
    switch (axis) {
        case (EAxis::X) :
            l = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_10_BIT);
            h = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_10_BIT + 1);
            break;
        case (EAxis::Y) :
            l = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::Y_OUT_L_10_BIT);
            h = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::Y_OUT_L_10_BIT + 1);
            break;
        case (EAxis::Z) :
            l = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::Z_OUT_L_10_BIT);
            h = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::Z_OUT_L_10_BIT + 1);
            break;
    }
    return normalizationAxisToGValue(h, l);
}

void AccelerometerI2C::writeMctlValue(int value, int mask) {
    int val = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::MCTL) & mask;
    wiringPiI2CWriteReg8(this->accelerometer_file_description, ERegisters::MCTL, value & ~val);
}

bool AccelerometerI2C::writeMctlValueAndCheck(int value, int mask) {
    writeMctlValue(value, mask);
    int val = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::MCTL) & mask;
    return (val == value);
}
