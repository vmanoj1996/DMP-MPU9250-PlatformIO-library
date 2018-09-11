/*
These file is obtained from "MPU9250-dmp" Code(By mbedoguz-Oğuz Özdemir) containing the motiondriver 6.1(from Invensense) and Arduino Library for MPU(From sparkfun).

Description Provided in the Mbed site is given below for reference
MPU9250 library using dmp, it uses sparkfun arduino library and invensense Motion driver 6.1.
https://os.mbed.com/users/mbedoguz/code/MPU9250-dmp/

IMU device IIc address can be changed in the mdcompat.cpp file (first 2 lines)

*/
#ifndef MY_I2C_H_INCLUDED
#define MY_I2C_H_INCLUDED

/**
 * Compatiblity Declarations
 */
#define PI 3.1415926536
void counter();
void imu_init(void);
void stamper_init(void);
unsigned short constrain(unsigned short x,unsigned short a, unsigned short b);

#ifdef __cplusplus
extern "C" {
#endif

int mbed_i2c_write(
    unsigned char slave_addr,
    unsigned char reg_addr,
    unsigned char length,
    unsigned char *data);

int mbed_i2c_read(
    unsigned char slave_addr,
    unsigned char reg_addr,
    unsigned char length,
    unsigned char *data);

int delay_ms(
    unsigned long num_ms);

int get_ms(
    unsigned long *count);

int reg_int_cb(
    void (*cb)(void),
    unsigned char port,
    unsigned char pin);

long labs(long x);

float fabsf(float x);

int min(int a, int b);


#ifdef __cplusplus
}
#endif

#endif // MY_I2C_H_INCLUDED
