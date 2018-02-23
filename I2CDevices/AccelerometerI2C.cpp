//
// Created by dev on 2/19/18.
//

#include <iomanip>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "AccelerometerI2C.h"


using namespace std::chrono_literals;

AccelerometerI2C::AccelerometerI2C(int deviceAddress) : axis_data { 0, 0, 0 }, last_data { 0, 0, 0, 0, 0, 0 } {
    wiringPiSetup();
    accelerometer_file_description = wiringPiI2CSetup(deviceAddress);
    if (accelerometer_file_description == -1) {
        throw std::runtime_error("Can't setup the I2C device.");
    }
    for (int i = 0; i < 10; i++) {
        int data = wiringPiI2CReadReg8(accelerometer_file_description, (int)ERegisters::WHAT_IS_MY_ADDRESS);
        std::cout << std::hex << data << std::endl;
    }
}

void AccelerometerI2C::calibrate() {
    calibrate(5s);

}

void AccelerometerI2C::calibrate(std::chrono::milliseconds milliseconds) {

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

std::string AccelerometerI2C::toStringLastData() {
    return std::__cxx11::string();
}

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
    return 0;
}

void AccelerometerI2C::read10BitData(float &XYZ[]) {
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
