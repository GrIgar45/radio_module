//
// Created by dev on 2/19/18.
//

#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <iostream>
#include "AccelerometerI2C.h"


AccelerometerI2C::AccelerometerI2C(int deviceAddress) : axisData{0, 0, 0}, lastData {0, 0, 0, 0, 0, 0}  {
    wiringPiSetup();
    gyro = wiringPiI2CSetup(deviceAddress);
    if (gyro == -1) {
        throw std::runtime_error("Can't setup the I2C device.");
    }
    for (int i = 0; i < 10; i++) {
        int data = wiringPiI2CReadReg8(gyro, (int)reg::WHAT_IS_MY_ADDRESS);
        std::cout << std::hex << data << std::endl;
    }
}

void AccelerometerI2C::calibrate() {

}

void AccelerometerI2C::calibrate(std::chrono::milliseconds milliseconds) {

}

void AccelerometerI2C::stop() {

}

std::ostream &operator<<(std::ostream &s, const AccelerometerI2C &data) {
    return  s;
}

std::string AccelerometerI2C::toStringLastData() {
    return std::__cxx11::string();
}

int AccelerometerI2C::getX() const {
    return 0;
}

int AccelerometerI2C::getY() const {
    return 0;
}

int AccelerometerI2C::getZ() const {
    return 0;
}

float AccelerometerI2C::normalizationAxis(int H, int L) {
    return 0;
}

void AccelerometerI2C::readData() {

}
