//
// Created by dev on 2/9/18.
//

#ifndef I2C_GYROI2C_H
#define I2C_GYROI2C_H


#include <chrono>
#include <thread>


struct GyroI2C {

    GyroI2C(int deviceAddress);

    void calibrate();

    void calibrate(std::chrono::milliseconds milliseconds);

    void stop();

    friend std::ostream &operator<<(std::ostream &os, const GyroI2C &data);

    std::string toStringLastData();

    int getX()const;

    int getY()const;

    int getZ()const;

    float normalizationAxis(int H, int L);

private:
    enum reg {
        WHO_AM_I = 0x0f,
        GYRO_NAME = 0xd4,
        CTRL_REG1 = 0x20,
        CTRL_REG4 = 0x23,
        NORMAL_MODE = 0x0f,
        OUT_X_L = 0x28,
        IS_NEW_DATA_READY = 0x27,
        DATA_READY = 0x08
    };
    std::thread *reading;
    int axisData[3];
    float noiseData[3];
    int lastData[6];
    int gyro = -1;
    int calibrated = 0;
    bool run = false;

    void readData();

};


#endif //I2C_GYROI2C_H
