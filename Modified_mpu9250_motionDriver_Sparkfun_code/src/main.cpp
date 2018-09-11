/*This code is a modified sparkfun MPU9250_dmp code(ported to ARM by mbedoguz)
https://os.mbed.com/users/mbedoguz/code/SparkFunMPU9250-DMP/ released under Apache License
 Tested on st-nucleo f446Re with MPU9250.
 Coded using PlatformIO-Atom IDE.
 This code is based on MotionDriver 6.12 by Invensense

KNOWN ISSUEs:
 1)please do not use this code for performance critical application.
    The Motiondriver code given by invensense has delay_ms() functions in inv_mpu.c
    It might cause unwanted delays in the main code!!
 2)Magnetometer is not used by DMP for sensor fusion. So yaw reading might drift over time.

Please do Accelerometer calibration and set offsets first
It can be done by 2 ways
1)uncomment(if commented) #define DO_ACCEL_CALIBRATION and run this code. Log the data and find average
  Ideally we should get (0,0,1.0) for ax,ay,az. If Not subtract the obtained averages with expected numbers
  and set them as ACCEL_BIAS(after converting the format as given in set_accel_bias() ).
  comment #define DO_ACCEL_CALIBRATION  and the code is ready to give calibrated angles
2)Online calibration can be done by taking samples for first few seconds(when even the board gets powered on)
  subtract the average raw readings with expected raw readings(0,0,1) and set it as bias using set_accel_bias()

*/
#define DMP_FIFO_RATE 100
//#define DO_ACCEL_CALIBRATION
#ifndef DO_ACCEL_CALIBRATION
  #define USE_ACCEL_OFFSETS
#endif
//#define ENABLE_TEAPOT_OUTPUT
#include "mbed.h"
#include "SparkFunMPU9250-DMP.h"
#include "math.h"
Serial pc(USBTX,USBRX);
DigitalOut led(LED1);

MPU9250_DMP imu;
unsigned char whoami[1]={0};
unsigned char fifo_count[2]={32,32};
unsigned char temp[1]={32};
unsigned char outbuff[4];
unsigned char inbuff[4];
unsigned char regadd;
char registeradress[5];
unsigned char registerdata[]={33};
void printIMUData(void);
void normalise(float* quat0,float* quat1 ,float* quat2 ,float* quat3);
int self_test_imu();
int set_accel_bias(long *);
void MPU_teaPot();
long ACCEL_BIAS[3] = {249, 246, -366};
#ifndef PI
#define PI 3.14159265
#endif
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
Timer t;
    int main()
    {
      t.start();
      pc.baud(115200);
      pc.printf("Hello World\n");
      imu_init();
      stamper_init();

      // Call imu.begin() to verify communication and initialize
      if (imu.begin() != INV_SUCCESS) {
          while (1){  pc.printf("Unable to communicate with MPU-9250");
                      pc.printf("Check connections, and try again.\n");
                      wait_ms(5000); }  }

        pc.printf("imu.begin()-IMU comm & initialize completed \n");
          #ifdef USE_ACCEL_OFFSETS
            int accel_bias_OUT= set_accel_bias(ACCEL_BIAS);
            if(accel_bias_OUT!=0){pc.printf("cupped-calibration\n");while(1){;}}//dead
          #endif

        if(imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL | \
          DMP_FEATURE_SEND_CAL_GYRO,DMP_FIFO_RATE)==INV_ERROR)
          {
            pc.printf("imu.dmpBegin have failed\n");

            /*mpLoad function under it fails which is caused by memcmp
            firmware+ii, cur, this_write) (it is located at row 2871 of inv_mpu.c)
            */
          }
        else{
            pc.printf("imu.dmpBegin() suceeded\n");
            wait(3);
            }
        pc.printf("$GETEU,*\n");

        while(1){//loop
      // Check for new data in the FIFO
      if (imu.fifoAvailable()){//fifo is not being available
        led=0;
        wait_ms(1);
        led=1;
        wait_ms(1);
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS){
          // computeEulerAngles can be used -- after updating the
          // quaternion values -- to estimate roll, pitch, and yaw
          imu.computeEulerAngles();
          #ifndef ENABLE_TEAPOT_OUTPUT
          printIMUData();
          #endif
          #ifdef ENABLE_TEAPOT_OUTPUT
          MPU_teaPot();
          #endif
        }
      }
      else{
              led=0;
      }
    }
      return 0;
    }


  int self_test_imu(){
    pc.baud(115200);
    pc.printf("Hello World\n");
    imu_init();
    stamper_init();

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS){
      while (1){
        pc.printf("Unable to communicate with MPU-9250");
        pc.printf("Check connections, and try again.\n");
        wait_ms(5000);
      }
    }
      pc.printf("imu.begin() suceeded\n");
  int abc= imu.selfTest();
  wait(10);
  pc.printf("self test results %d\n",abc);
  // its a bit mask 111 or 7 means gyro,acc,mag are all fine
  return 0;
  }

    void normalise(float* quat0,float* quat1 ,float* quat2 ,float* quat3)
    {
      float normy = sqrt( (*quat0)*(*quat0)+(*quat1)*(*quat1)+(*quat2)*(*quat2)+(*quat3)*(*quat3) );
      *quat0=(*quat0)/normy;
      *quat1=(*quat1)/normy;
      *quat2=(*quat2)/normy;
      *quat3=(*quat3)/normy;
      return;
    }
    void decompose_quat(float quat0, float quat1, float quat2, float quat3, float * quat_angle, float quat_axis[])
    {

      *quat_angle = 2*acos(quat0) * 180.0/PI;//returns angle in [0,180];
      quat_axis[0]= quat1/sin(*quat_angle*PI/180.0*0.5);//This  maynot be the right way!!
      quat_axis[1]= quat2/sin(*quat_angle*PI/180.0*0.5);//use sqrt() instead of sin
      quat_axis[2]= quat3/sin(*quat_angle*PI/180.0*0.5);
      return;
    }
    void printIMUData(void)
    {
      // After calling dmpUpdateFifo() the ax, gx, mx, etc. values
      // are all updated.
      // Quaternion values are, by default, stored in Q30 long
      float q0 = imu.calcQuat(imu.qw);
      float q1 = imu.calcQuat(imu.qx);
      float q2 = imu.calcQuat(imu.qy);
      float q3 = imu.calcQuat(imu.qz);
      normalise(&q0,&q1,&q2,&q3);

      float acc[3]={0.0,0.0,0.0},gyr[3] = {0,0,0};
      acc[0] = imu.calcAccel(imu.ax);//units= g; meaning 9.81 will be shown as 1g ideally
      acc[1] = imu.calcAccel(imu.ay);
      acc[2] = imu.calcAccel(imu.az);
      gyr[0] = imu.calcGyro(imu.gx);
      gyr[1] = imu.calcGyro(imu.gy);
      gyr[2] = imu.calcGyro(imu.gz);
      pc.printf("%d ",t.read_ms());
      pc.printf("%.4lf %.4lf %.4lf \n", imu.yaw,imu.pitch,imu.roll);

    }
int set_accel_bias(long bias[]){
  /*This function accepts the bias for calibration in LSB format in +-8g FSR.
   Meaning 1g offset should be given as (1/8*2^15)LongInt
   calls mpu_set_accel_bias_6500_reg function from inv_mpu.c(motiondriver 6.1 API).
   BUT it is not working that way. If z acc is 0.8 and 1g is our expectation,
   Then we should take (-0.2*2^15/8)/2 to get the bias :(
   Honestly dont know why it is working like this
   */
  int a = -1;
  a=mpu_set_accel_bias_6500_reg(bias);
  return a; //0-successful :)
}
void MPU_teaPot()
{
            float q0 = imu.calcQuat(imu.qw);
            float q1 = imu.calcQuat(imu.qx);
            float q2 = imu.calcQuat(imu.qy);
            float q3 = imu.calcQuat(imu.qz);
            int16_t q_int[4] = { q0*16384, q1*16384, q2*16384, q3*16384 };
            teapotPacket[2] = q_int[0]>>8;//msbs of q0
            teapotPacket[3] = q_int[0] & 0x00FF;//lsbs of q0
            teapotPacket[4] = q_int[1]>>8;
            teapotPacket[5] = q_int[1] & 0x00FF;
            teapotPacket[6] = q_int[2]>>8;
            teapotPacket[7] = q_int[2] & 0x00FF;
            teapotPacket[8] = q_int[3]>>8;
            teapotPacket[9] = q_int[3] & 0x00FF;

            for(int i=0;i<14;i++){
              pc.putc(teapotPacket[i]);
            }
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
}
