#pragma once
#include <Arduino.h>
#include <Common/Common.h>
#include <Task/Task.h>
#include <Wire.h>


// include I2C MPU headers
// #define __PGMSPACE_H_   1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define MPU_MODE_RAW  0
#define MPU_MODE_DMP  1
#define DEFAULT_CORE  0 // main loop default on 0, keep this to spare core 

/**********************************************************************************************
 
  Wrapper class for Gyroscope
 



 
 *********************************************************************************************/
 
struct gyro_config_t {
  int   i2c_address;
  int   i2c_scl_pin;
  int   i2c_sda_pin;       
  int   i2c_clock;       
  int   dmp_int_pin;
  int   core;
};

class Gyro  : public Task {
  public:
    static Gyro * instance;
    Gyro(gyro_config_t cfg);
    void init();
    void run(void* data);
    static void  dmpDataReady () {
      Gyro::instance->mpuInterrupt = true;
    } ;

    volatile bool mpuDataReady = false;
    volatile bool mpuInterrupt = false;   
    float * getYPR(void);

  private:
    gyro_config_t   config;
    MPU6050*        mpu;
    int             mode;
    bool            dmpReady = false;   // set true if DMP init was successful
    uint8_t         IntStatus;          // holds actual interrupt status byte from MPU
    uint8_t         devStatus;          // return status after each device operation (0 = success, !0 = error)
    uint16_t        packetSize;         // expected DMP packet size (default is 42 bytes)
    uint16_t        fifoCount;          // count of all bytes currently in FIFO
    uint8_t         fifoBuffer[64];     // FIFO storage buffer     
    Quaternion      q;                  // [w, x, y, z]         quaternion container
    //VectorInt16 aa;               // [x, y, z]            accel sensor measurements
    //VectorInt16 aaReal;           // [x, y, z]            gravity-free accel sensor measurements
    //VectorInt16 aaWorld;          // [x, y, z]            world-frame accel sensor measurements
    VectorFloat     gravity;            // [x, y, z]            gravity vector
    float           euler[3];                 // [psi, theta, phi]    Euler angle container
    float           ypr[3];                   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector    
    
    volatile int mpuDataCounter = 0;
    int mpuDataCounterPrev = 0;
};

