//
// Created by dev on 2/19/18.
//

#ifndef I2C_ACCELEROMETER_H
#define I2C_ACCELEROMETER_H


#include <chrono>
#include <ostream>
#include <thread>


struct AccelerometerI2C {
    AccelerometerI2C(int deviceAddress);

    void calibrate();

    void calibrate(std::chrono::milliseconds milliseconds);

    void stop();

    friend std::ostream &operator<<(std::ostream &os, const AccelerometerI2C &data);

    std::string toStringLastData();

    float getX() const;

    float getY() const;

    float getZ() const;

private:

    enum ERegisters {
        WHAT_IS_MY_ADDRESS = 0x0d,
        WHO_AM_I = 0x0f,
        GYRO_NAME = 0xd4,
        CTRL_REG1 = 0x20,
        CTRL_REG4 = 0x23,
        NORMAL_MODE = 0x0f,
        X_OUT_L_10_BIT = 0x00,
        Y_OUT_L_10_BIT = 0x02,
        Z_OUT_L_10_BIT = 0x04,
    };
    enum class EAxis { X, Y, Z };
    std::thread *reading;
    float axis_data[3];
    float noise_data[3];
    float last_data[3];
    int sensitivity;

    int accelerometer_file_description = -1;
    bool calibrated = false;
    bool run = false;

    void setAxisOffset(int x, int y, int z);

    void readingLoop();

    float normalizationAxisToGValue(int high_byte, int low_byte);

    void read10BitData(float &XYZ[]);

    float read10BitData(EAxis axis);
};


#endif //I2C_ACCELEROMETER_H
