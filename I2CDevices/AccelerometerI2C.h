//
// Created by dev on 2/19/18.
//

#ifndef I2C_ACCELEROMETER_H
#define I2C_ACCELEROMETER_H


#include <chrono>
#include <ostream>
#include <thread>


struct AccelerometerI2C {
    enum class ESensitivity {
        G2, G4, G8
    };

    enum class EMode {
        STANDBY, MEASURE, LEVEL, PULSE
    };

    AccelerometerI2C(int deviceAddress);

    void calibrate();

    void calibrate(std::chrono::milliseconds milliseconds);

    bool setSensitivity(ESensitivity sensitivity);

    bool setMode(EMode mode);

    bool setSensitivityAndMode(ESensitivity sensitivity, EMode mode);

    void stop();

    friend std::ostream &operator<<(std::ostream &os, const AccelerometerI2C &data);

//    std::string toStringLastData();

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
        MCTL = 0x16,
        MCTL_GLVL_MASK = 0x0C,
        MCTL_2G = 0x01,
        MCTL_4G = 0x02,
        MCTL_8G = 0x00,
        MCTL_MODE_MASK = 0x03,
        MCTL_STANDBY = 0x00,
        MCTL_MEASUMENT = 0x01,
        MCTL_LEVEL = 0x02,
        MCTL_PULSE = 0x03,
        X_OUT_L_10_BIT = 0x00,
        Y_OUT_L_10_BIT = 0x02,
        Z_OUT_L_10_BIT = 0x04,
        HIGH_BIT_MASK = 0x03,
        LOW_BIT_MASK = 0xFF,
        MINUS_10_BIT_MASK = (-1 & ~0x3FF),
        MINUS_8_BIT_MASK = (-1 & ~0xFF)
    };
    enum class EAxis { X, Y, Z };
    std::thread *reading;
    float axis_data[3];
    float noise_data[3];
//    float last_data[3];
    int sensitivity;

    int accelerometer_file_description = -1;
    bool calibrated = false;
    bool run = false;

    void writeMctlValue(int value, int mask);

    bool writeMctlValueAndCheck(int value, int mask);

    void setAxisOffset(int x, int y, int z);

    void readingLoop();

    enum class ENormalizeType {
        bit10, bit16
    };
    float normalizationAxisToGValue(int high_byte, int low_byte, ENormalizeType type);

    void read10BitData(float *XYZ);

    float read10BitData(EAxis axis);

    void read16BitData(float *XYZ);

    float read16BitData(EAxis axis);
};


#endif //I2C_ACCELEROMETER_H
