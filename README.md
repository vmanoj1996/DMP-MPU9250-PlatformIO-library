# DMP-MPU9250-PlatformIO-library
Library for interfacing MPU9250(with DMP) with ST-Nucleo Boards.

The code can be used for different invensense IMUs by modifying inv_mpu.c(line 130).

Derived from Oğuz Özdemir's work(with some corrections)

Sparkfun's Arduino library was customized for Mbed by Oğuz Özdemir https://os.mbed.com/users/mbedoguz/code/MPU9250-dmp/
uses Invensense motion driver 6.1. 

**Note**:
Invensense driver has delay functions in it. Not to be used for real time & safety critical applications.
