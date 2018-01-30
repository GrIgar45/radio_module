#include <stdio.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

#define MPU6050_GYRO_XOUT_H        0x43   // R
#define MPU6050_GYRO_YOUT_H        0x45   // R
#define MPU6050_GYRO_ZOUT_H        0x47   // R

#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_I2C_ADDRESS        0x68   // I2C

int main()
{
    int fd = wiringPiI2CSetup(MPU6050_I2C_ADDRESS);
    if (fd == -1)
        return;

    wiringPiI2CReadReg8 (fd, MPU6050_PWR_MGMT_1);
    wiringPiI2CWriteReg16(fd, MPU6050_PWR_MGMT_1, 0);

    float x,y,z;

    while(true)
    {
        x = wiringPiI2CReadReg8(fd, MPU6050_GYRO_XOUT_H);
        y = wiringPiI2CReadReg8(fd, MPU6050_GYRO_YOUT_H);
        z = wiringPiI2CReadReg8(fd, MPU6050_GYRO_ZOUT_H);

        printf("x=%d   y=%d   z=%d", x,y,z); 
    }
}
