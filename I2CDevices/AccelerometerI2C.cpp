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

AccelerometerI2C::AccelerometerI2C(int deviceAddress) {
    wiringPiSetup();
    accelerometer_file_description = wiringPiI2CSetup(deviceAddress);
    if (accelerometer_file_description == -1) {
        throw std::runtime_error("Can't setup the I2C device.");
    }
    this->setSensitivityAndMode(ESensitivity::G2, EMode::MEASURE);
}

void AccelerometerI2C::calibrate(int x, int y, int z) {
    calibrate(5s, x, y, z);
}

void AccelerometerI2C::calibrate(std::chrono::milliseconds milliseconds, int x, int y, int z) {
    setAxisOffset(x, y, z);
#ifndef NDEBUG
    auto xyz = getAxisOffset();
    std::cout << "Offset is ";
    for (auto offset : xyz) {
        std::cout << offset << " : ";
    }
    std::cout << std::endl;
#endif
    this->run = true;
    this->reading = new std::thread(&AccelerometerI2C::readingLoop, this);
    std::this_thread::sleep_for(2s);
//    milliseconds = milliseconds - 2s;
//    auto start = std::chrono::steady_clock::now();
//    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start) <
//           milliseconds) {
//        for (int i = 0; i < 3; i++) {
//            auto d = std::abs(last_data[i]);
//            noise_values[i] = (d > noise_values[i] && d < 10.0f) ? d : noise_values[i];
//        }
//        std::this_thread::sleep_for(
//                std::chrono::duration_cast<std::chrono::milliseconds>(10ms));
//    }
#ifndef NDEBUG
    std::stringstream s;
    s << std::fixed << std::setprecision(3);
    s << "Calibration successful. X: " << noise_data[0] << " Y: " << noise_data[1] << " Z: " << noise_data[2]
      << std::endl;
    std::cout << s.str();
#endif
    this->calibrated = 1;
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
    return this->writeMctlValueAndCheck(selected << 2, ERegisters::MCTL_GLVL_MASK);
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
    return setMode(mode) && setSensitivity(sensitivity);
}

void AccelerometerI2C::stop() {
    if (!this->run) { return; }
    this->run = false;
    this->reading->join();
    delete this->reading;
}

std::ostream &operator<<(std::ostream &ostream, const AccelerometerI2C &data) {
    std::stringstream stringstream;
    stringstream << std::fixed;
    stringstream << "Position\n";
    stringstream << std::setfill(' ');
    stringstream << "X: " << std::setw(7) << std::setprecision(3) << data.getX();
    stringstream << "\tY: " << std::setw(7) << std::setprecision(3) << data.getY();
    stringstream << "\tZ: " << std::setw(7) << std::setprecision(3) << data.getZ();
    stringstream << std::endl;
    stringstream << "Gravity\n";
    stringstream << "X: " << std::setw(7) << data.convertToGForce(data.gravity[0]);
    stringstream << "\tY: " << std::setw(7) << data.convertToGForce(data.gravity[1]);
    stringstream << "\tZ: " << std::setw(7) << data.convertToGForce(data.gravity[2]);
    ostream << stringstream.str();
    return ostream;
}

float AccelerometerI2C::getX() const {
    static const auto index = 0;
    auto ret = convertToGForce(last_data[index]) - convertToGForce(gravity[index]);
    return ret;
}

float AccelerometerI2C::getY() const {
    static const auto index = 1;
    auto ret = convertToGForce(last_data[index]) - convertToGForce(gravity[index]);
    return ret;
}

float AccelerometerI2C::getZ() const {
    static const auto index = 2;
    auto ret = convertToGForce(last_data[index]) - convertToGForce(gravity[index]);
    return ret;
}

void AccelerometerI2C::setAxisOffset(int x, int y, int z) {
    const auto MASK_LOW_BIT = 0xfF;
    const auto MASK_HIGH_BIT = 0x7;
    auto i = 0;
    for (auto xyz : { x, y, z }) {
        wiringPiI2CWriteReg8(this->accelerometer_file_description, ERegisters::XOFFL + (i << 1), xyz & MASK_LOW_BIT);
        wiringPiI2CWriteReg8(this->accelerometer_file_description, ERegisters::XOFFL + (i << 1) + 1,
                             (xyz >> 8) & MASK_HIGH_BIT);
        ++i;
    }
}

std::array<int, 3> AccelerometerI2C::getAxisOffset() {
    std::array<int, 3> offset_data { 0, 0, 0 };
    for (int i = 0; i < 3; i++) {
        auto l = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::XOFFL + (i << 1));
        auto h = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::XOFFL + (i << 1) + 1);
        offset_data[i] = (h << 8) | l;
    }
    return offset_data;
}

void AccelerometerI2C::readingLoop() {
    const auto n = 3;

    while (run) {
        read8BitData(last_data);
        std::this_thread::sleep_for(3ms);
    }
}

AccelerometerI2C::GData AccelerometerI2C::normalization10BitsAxisToGValue(int high_byte, int low_byte) {
    auto sign = ((high_byte & 0b10) == 0) ? 1 : -1;
    high_byte &= ERegisters::BIT_2_MASK;
    low_byte &= ERegisters::BIT_8_MASK;

    int number = (high_byte << 8) | low_byte;

    if (number == 0) { return 0; }

    if (sign == -1) { number |= ERegisters::MINUS_10_BIT_MASK; }

    return number;
}

AccelerometerI2C::GData AccelerometerI2C::normalization8BitsAxisToGValue(int byte) {
    auto sign = ((byte & 128) == 0) ? 1 : -1;
    byte &= ERegisters::BIT_8_MASK;

    if (byte == 0) { return 0; }

    if (sign == -1) { byte |= ERegisters::MINUS_8_BIT_MASK; }

    return byte;
}
//                                                              1.5  96             1.0  64
//                                                              0.3  19             0.8  51
//                                                                                  1.1  70
float AccelerometerI2C::updateGravity(AccelerometerI2C::GData axis_value, float old_gravity) {
    const auto alpha = 0.8f;
    return (alpha * old_gravity + (1 - alpha) * axis_value);
}

inline float AccelerometerI2C::convertToGForce(AccelerometerI2C::GData value) const {
    return convertToGForce((float)(value));
}

inline float AccelerometerI2C::convertToGForce(float value) const {
    return (value) / (128 / this->sensitivity);
}

void AccelerometerI2C::writeMctlValue(int value, int mask) {
    auto val = (wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::MCTL) & ~mask) | value;
    wiringPiI2CWriteReg8(this->accelerometer_file_description, ERegisters::MCTL, val);
}

bool AccelerometerI2C::writeMctlValueAndCheck(int value, int mask) {
    writeMctlValue(value, mask);
    auto val = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::MCTL) & mask;
    return (val == value);
}

void AccelerometerI2C::read10BitData(GData *xyz) {
    const int n = 3;
    for (int i = 0; i < n; ++i) {
        auto l = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_10_BIT + (i << 1));
        auto h = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_10_BIT + (i << 1) + 1);
        xyz[i] = normalization10BitsAxisToGValue(h, l);
        gravity[i] = updateGravity(xyz[i], gravity[i]);
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
    return normalization10BitsAxisToGValue(h, l);
}

void AccelerometerI2C::read8BitData(GData *xyz) {
    const int N = 3;
    for (int i = 0; i < N; ++i) {
        auto val = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_8_BIT + i);
        val = normalization8BitsAxisToGValue(val);
//        val = (std::abs(val) > noise_values[i]) ? val : 0;
        gravity[i] = updateGravity(val, gravity[i]);
        xyz[i] = val;
    }
}

float AccelerometerI2C::read8BitData(AccelerometerI2C::EAxis axis) {
    int val;
    switch (axis) {
        case (EAxis::X) :
            val = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::X_OUT_L_8_BIT);
            break;
        case (EAxis::Y) :
            val = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::Y_OUT_L_8_BIT);
            break;
        case (EAxis::Z) :
            val = wiringPiI2CReadReg8(this->accelerometer_file_description, ERegisters::Z_OUT_L_8_BIT);
            break;
    }
    return normalization8BitsAxisToGValue(val);
}


