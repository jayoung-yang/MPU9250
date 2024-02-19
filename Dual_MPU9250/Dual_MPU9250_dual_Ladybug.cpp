/* 07/6/2017 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer
 * 
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor, 
 getting properly scaled accelerometer, gyroscope, and magnetometer data out. 
 Addition of 9 DoF sensor fusion using open source Madgwick filter algorithm. 

 Sketch modified to read data from two MPU9250s, one at 0x68 and 0ne at 0x69, on the same I2C bus and get absolute orientation
 Sketch runs on the 3.3 V Ladybug STM32L432 Breakout Board.
 
 Library may be used freely and without limit with attribution.
*/
 
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <fstream>
#include <chrono>
#include <thread>

using namespace std;

#include "MPU9250.h"

#define SerialDebug false   // set to true to get Serial output for debugging


// RTC set time using STM32L4 natve RTC class
/* Change these values to set the current initial time */
const uint8_t seconds = 0;
const uint8_t minutes = 15;
const uint8_t hours = 17;

/* Change these values to set the current initial date */
const uint8_t day = 4;
const uint8_t month = 7;
const uint8_t year = 17;

uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false; // for RTC alarm interrupt


// MPU9250 Configuration
// Specify sensor full scale
/* Choices are:
 *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
 *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
 *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
 *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
 *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so 
 *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x00;         
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
float motion = 0; // check on linear acceleration to determine motion
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
bool wakeup;

// Pin definitions
int  intPin1 = 9;  //  MPU9250 1 interrupt
int  intPin2 = 8;  //  MPU9250 2 interrupt
int  myLed  = 13; // red led

bool intFlag1 = false;
bool intFlag2 = false;
bool newMagData = false;

int16_t MPU9250Data1[7], MPU9250Data2[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount1[3], magCount2[3];    // Stores the 16-bit signed magnetometer sensor output
float   magCalibration1[3] = {0, 0, 0}, magCalibration2[3] = {0, 0, 0};  // Factory mag calibration and mag bias
float   temperature1, temperature2;    // Stores the MPU9250 internal chip temperature in degrees Celsius
float   SelfTest[6];    // holds results of gyro and accelerometer self test

// These can be measured once and entered here or can be calculated each time the device is powered on
float   gyroBias1[3] = {0.96, -0.21, 0.12}, accelBias1[3] = {0.00299, -0.00916, 0.00952};
float   gyroBias2[3] = {0.96, -0.21, 0.12}, accelBias2[3] = {0.00299, -0.00916, 0.00952};
float   magBias1[3] = {71.04, 122.43, -36.90}, magScale1[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer
float   magBias2[3] = {71.04, 122.43, -36.90}, magScale2[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer


uint32_t delt_t1, delt_t2 = 0;                      // used to control display output rate
uint32_t count1 = 0, sumCount1 = 0, count2 = 0, sumCount2 = 0;         // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;                   // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33;            // rotation matrix coefficients for Euler angles and gravity components
float deltat1 = 0.0f, sum1 = 0.0f, deltat2 = 0.0f, sum2 = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, lastUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                         // used to calculate integration interval

float ax1, ay1, az1, gx1, gy1, gz1, mx1, my1, mz1; // variables to hold latest sensor data values 
float ax2, ay2, az2, gx2, gy2, gz2, mx2, my2, mz2; // variables to hold latest sensor data values 
float lin_ax1, lin_ay1, lin_az1;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float lin_ax2, lin_ay2, lin_az2;             // linear acceleration (acceleration with gravity component subtracted)
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion

MPU9250 MPU9250(intPin1); // instantiate MPU9250 class

ofstream ofile;


void setup()
{
  
  MPU9250.I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71 
  

  // pinMode(intPin1, INPUT);

   /* Configure the MPU9250 */
  // Read the WHO_AM_I register, this is a good test of communication
  printf("MPU9250 9-axis motion sensor...\n");
  uint8_t c = MPU9250.getMPU9250ID();
  printf("MPU9250_1 I AM %x I should be %x\n", c, 0x71);
  usleep(1 * 1000);
  
  if (c == 0x71) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
  {  
    printf("MPU9250 1 is online...\n");
    
    MPU9250.resetMPU9250(); // start by resetting MPU9250_1
    
    MPU9250.SelfTest(SelfTest); // Start by performing self test and reporting values
    printf("Self Test for MPU9250 #1:\n");
    printf("x-axis self test: acceleration trim within : %f%% of factory value\n", SelfTest[0]); 
    printf("y-axis self test: acceleration trim within : %f%% of factory value\n", SelfTest[1]);
    printf("z-axis self test: acceleration trim within : %f%% of factory value\n", SelfTest[2]);
    printf("x-axis self test: gyration trim within : %f%% of factory value\n", SelfTest[3]);
    printf("y-axis self test: gyration trim within : %f%% of factory value\n", SelfTest[4]);
    printf("z-axis self test: gyration trim within : %f%% of factory value\n", SelfTest[5]);
    usleep(1 * 1000);

  // get sensor resolutions, only need to do this once, same for both MPU9250s for now
  aRes = MPU9250.getAres(Ascale);
  gRes = MPU9250.getGres(Gscale);
  // mRes = MPU9250.getMres(Mscale);

 // Comment out if using pre-measured, pre-stored offset biases
  // MPU9250.calibrateMPU9250(gyroBias1, accelBias1); // Calibrate gyro and accelerometers, load biases in bias registers
  // printf("MPU1 accel biases (mg)\n"); printf("%f, %f, %f\n", 1000.*accelBias1[0], 1000.*accelBias1[1], 1000.*accelBias1[2]);
  // printf("MPU1 gyro biases (dps)\n"); printf("%f, %f, %f\n", gyroBias1[0], gyroBias1[1], gyroBias1[2]);
  // usleep(1 * 1000); 
  
  MPU9250.initMPU9250(Ascale, Gscale, sampleRate); 
  printf("MPU9250 1 is initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
  
  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  // char e = MPU9250.getAK8963CID();  // Read WHO_AM_I register for AK8963
  // printf("AK8963 1 I AM %x I should be %x\n", e, 0x48);
  // usleep(1 * 1000); 
  
  // Get magnetometer calibration from AK8963 ROM
  // MPU9250.initAK8963Slave(Mscale, Mmode, magCalibration1); printf("AK8963 1 initialized for active data mode....\n"); // Initialize device 1 for active mode read of magnetometer
  // printf("Calibration values for mag 1: \n");
  // printf("X-Axis sensitivity adjustment value %f\n", magCalibration1[0]);
  // printf("Y-Axis sensitivity adjustment value %f\n", magCalibration1[1]);
  // printf("Z-Axis sensitivity adjustment value %f\n", magCalibration1[2]);
  
 // Comment out if using pre-measured, pre-stored offset biases
  // MPU9250.magcalMPU9250(magBias1, magScale1);
  // printf("AK8963 1 mag biases (mG) %f, %f, %f\n", magBias1[0], magBias1[1], magBias1[2]); 
  // printf("AK8963 1 mag scale (mG) %f, %f, %f\n", magScale1[0], magScale1[1], magScale1[2]); 
  // usleep(2 * 1000); // add delay to see results before serial spew of data
 
  
  // attachInterrupt(intPin1, myinthandler1, RISING);  // define interrupt for intPin output of MPU9250 1

  
  }
  else
  {
    printf("Could not connect to MPU9250 1: %x\n", c);
    while(1) ; // Loop forever if communication doesn't happen
  }


  
  /* Set up the RTC alarm interrupt */
  // RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  // RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds
  
}

struct timespec ts;
unsigned long nsec;

void loop()
{  
   // If intPin1 goes high, either all data registers have new data
  //  if(intFlag1 == true) {   // On interrupt, read data
      // intFlag1 = false;     // reset newData flag

    int ready = MPU9250.checkNewAccelGyroData();
    if (!ready) return;

    
      
    MPU9250.readMPU9250Data(MPU9250Data1); // INT cleared on any read
   
    // Now we'll calculate the accleration value into actual g's
     ax1 = (float)MPU9250Data1[0]*aRes - accelBias1[0];  // get actual g value, this depends on scale being set
     ay1 = (float)MPU9250Data1[1]*aRes - accelBias1[1];   
     az1 = (float)MPU9250Data1[2]*aRes - accelBias1[2];  

    // Calculate the gyro value into actual degrees per second
     gx1 = (float)MPU9250Data1[4]*gRes;  // get actual gyro value, this depends on scale being set
     gy1 = (float)MPU9250Data1[5]*gRes;  
     gz1 = (float)MPU9250Data1[6]*gRes; 
  
  //  if( MPU9250.checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
  //     MPU9250.readMagData(magCount1);  // Read the x/y/z adc values
  
  //   // Calculate the magnetometer values in milliGauss
  //   // Include factory calibration per data sheet and user environmental corrections
  //     mx1 = (float)magCount1[0]*mRes*magCalibration1[0] - magBias1[0];  // get actual magnetometer value, this depends on scale being set
  //     my1 = (float)magCount1[1]*mRes*magCalibration1[1] - magBias1[1];  
  //     mz1 = (float)magCount1[2]*mRes*magCalibration1[2] - magBias1[2];  
  //     mx1 *= magScale1[0];
  //     my1 *= magScale1[1];
  //     mz1 *= magScale1[2]; 
  //  }
   
  
    // for(uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
    // Now1 = micros();
    // deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
    // lastUpdate1 = Now1;

    // sum1 += deltat1; // sum for averaging filter update rate
    // sumCount1++;

    // MadgwickQuaternionUpdate1(-ax1, +ay1, +az1, gx1*pi/180.0f, -gy1*pi/180.0f, -gz1*pi/180.0f,  my1,  -mx1, mz1);
    // }

    /* end of MPU9250 1 interrupt handling */
  //  }
    
    ofile << nsec << ", " << ax1 << ", " << ay1 << ", " << az1 << ", " << gx1 << ", " << gy1 << ", " << gz1 << "\n";
    // printf("timestamp: %f\n", nsec / 1000000.0);
    // printf("ax: %f ay: %f az: %f g\n", ax1, ay1, az1);  
    // printf("gx: %f gy: %f gz: %f deg/s\n", gx1, gy1, gz1); 
    // printf("mx: %d my: %d mz: %d mG\n", (int)mx1, (int)my1, (int)mz1); 
    // printf("q0: %f qx: %f qy: %f qz: %f\n", q[0], q[1], q[2], q[3]);

    // temperature1 = ((float) MPU9250Data1[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    // printf("Gyro 1 temperature is %f degrees C\n", temperature1); // Print T values to tenths of s degree C
   
    // a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    // a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    // a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    // a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    // a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    // pitch1 = -asinf(a32);
    // roll1  = atan2f(a31, a33);
    // yaw1   = atan2f(a12, a22);
    // pitch1 *= 180.0f / pi;
    // yaw1   *= 180.0f / pi; 
    // yaw1   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    // if(yaw1 < 0) yaw1   += 360.0f; // Ensure yaw stays between 0 and 360
    // roll1  *= 180.0f / pi;
    // lin_ax1 = ax1 + a31;
    // lin_ay1 = ay1 + a32;
    // lin_az1 = az1 - a33;

    // print("MPU9250 1 Yaw, Pitch, Roll: %f, %f, %f\n", yaw1, pitch1, roll1);

    // print("Grav_x, Grav_y, Grav_z: %f, %f, %f mg\n", -a31*1000.0f, -a32*1000.0f, a33*1000.0f);
    // print("Lin_ax, Lin_ay, Lin_az: %f, %f, %f mg\n", lin_ax1*1000.0f, lin_ay1*1000.0f, lin_az1*1000.0f);

    // print("rate 1 = %f Hz\n", (float)sumCount1/sum1);
    // sumCount1 = 0;

} /* end of alarm handling */


//===================================================================================================================
//====== Set of useful functions
//===================================================================================================================

void myinthandler1()
{
  intFlag1 = true;
}

void myinthandler2()
{
  intFlag2 = true;
}


void alarmMatch()
{
  alarmFlag = true;
}

int main(void)
{
  setup();

  ofile.open("accelgyro.data");

  auto startTime = std::chrono::steady_clock::now();
  std::chrono::seconds duration(60);

  while(1) {
    loop();

    auto currentTime = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime);
    if (elapsed >= duration) {
      printf("test done\n");
      break;
    }
  }

  ofile.close();

  return 0;
}