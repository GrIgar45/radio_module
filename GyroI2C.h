//
// Created by dev on 2/9/18.
//

#ifndef I2C_GYROI2C_H
#define I2C_GYROI2C_H


#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>


struct GyroI2C {

    GyroI2C(int deviceAddress, int axisData);

    void calibrate();

    void calibrate(std::chrono::milliseconds milliseconds);

    void stop();

    std::string toString();

    int getX();

    int getY();

    int getZ();

    float normalizationAxis(int H, int L);

private:
    enum class reg {
        WHO_AM_I = 0x0f,
        GYRO_NAME = 0xd4,
        CTRL_REG1 = 0x20,
        CTRL_REG4 = 0x23,
        NORMAL_MODE = 0x0f,
        OUT_X_L = 0x28
    };
    std::thread *reading;
    std::mutex affordable;
    int axisData[3];
    float noiseData[3];
    int gyro = -1;
    int calibrated = 0;
    bool run = false;

    void readData();

};


#endif //I2C_GYROI2C_H
