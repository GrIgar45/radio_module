#include <cstdio>
#include <wiringPi.h>
#include <wiringPiI2C.h>

int main (int argc, char *argv[])
{
    int fd;
    int data;
    wiringPiSetup () ;
    fd=wiringPiI2CSetup (0x6b) ;  /*Use i2cdetect command to find your respective device address*/
    if(fd==-1)
    {
        printf("Can't setup the I2C device\n");
        return -1;
    } else {
        for (;;)
        {
            data=wiringPiI2CRead(fd);
            if(data==-1)
            {
                printf("No data\r");
                //return -1;
            } else {
                //print data
                printf("data=%d\r", data);
            }
        }
    }
    return 0;
}