#pragma once
#include <Arduino.h>


#define __PGMSPACE_H_   1 // stop compile errors of redefined typedefs and defines with ESP32-Arduino
// include I2C MPU headers
#include <Wire.h>
#include <Task/Task.h>
#include <I2Cdev.h>

#include <MPU6050_6Axis_MotionApps20.h>

#define MPU_MODE_RAW  0
#define MPU_MODE_DMP  1

/**********************************************************************************************
 *
 *  Wrapper class for Gyro MPU6050
 *
 *********************************************************************************************/
 
struct GyroConfig {
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
    Gyro(GyroConfig cfg);
    void init();
    void run(void* data);
    static void  dmpDataReady () {
      Gyro::instance->mpuInterrupt = true;
    } ;

    volatile bool mpuDataReady = false;
    volatile bool mpuInterrupt = false;   
    float * getYPR(void);

  private:
    GyroConfig  config;
    MPU6050*    mpu;
    int         mode;
    bool        dmpReady = false;   // set true if DMP init was successful
    uint8_t     IntStatus;          // holds actual interrupt status byte from MPU
    uint8_t     devStatus;          // return status after each device operation (0 = success, !0 = error)
    uint16_t    packetSize;         // expected DMP packet size (default is 42 bytes)
    uint16_t    fifoCount;          // count of all bytes currently in FIFO
    uint8_t     fifoBuffer[64];     // FIFO storage buffer     
    Quaternion  q;                  // [w, x, y, z]         quaternion container
    //VectorInt16 aa;               // [x, y, z]          accel sensor measurements
    //VectorInt16 aaReal;           // [x, y, z]          gravity-free accel sensor measurements
    //VectorInt16 aaWorld;          // [x, y, z]          world-frame accel sensor measurements
    VectorFloat gravity;            // [x, y, z]            gravity vector
    float euler[3];                 // [psi, theta, phi]    Euler angle container
    float ypr[3];                   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector    
    


    volatile int mpuDataCounter = 0;
    int mpuDataCounterPrev = 0;
};

Gyro::Gyro(GyroConfig cfg) {
  this->config = cfg;
  Task::setCore(cfg.core);
};


void Gyro::init() {
  
  this->mpu = new MPU6050(this->config.i2c_address);
  Wire.begin(this->config.i2c_sda_pin,this->config.i2c_scl_pin);
  Wire.setClock(this->config.i2c_clock); 
  delay(1000);
  this->mpu->initialize();
  delay(100);
  this->mpu->reset(); 
  delay(100);
  this->mpu->resetI2CMaster();
  delay(100);   
  this->mpu->initialize();
  pinMode(this->config.dmp_int_pin, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(this->mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));  
  
  
  Serial.println(F("Initializing DMP..."));
  this->devStatus = this->mpu->dmpInitialize();
  //this->mpu->setXGyroOffset(220);
  //this->mpu->setYGyroOffset(76);
  //this->mpu->setZGyroOffset(-85);
  //this->mpu->setZAccelOffset(1788);  
  
  if (this->devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    this->mpu->setDMPEnabled(true);  

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(this->config.dmp_int_pin, dmpDataReady, RISING);
    this->IntStatus = this->mpu->getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = this->mpu->dmpGetFIFOPacketSize();

  } else {
    Serial.println(F("DMP Initialization failed (code "));  Serial.println(devStatus);  Serial.println(F(")"));
  }
};


float * Gyro::getYPR() {
  return ypr;

}

void Gyro::run(void* data) {
  while (1) {
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        //
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
    }

    mpuInterrupt = false;
    this->IntStatus = this->mpu->getIntStatus();

    // get current FIFO count
    fifoCount = this->mpu->getFIFOCount();

    if (( this->IntStatus & 0x10) || fifoCount >= 1024) {
        this->mpu->resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } else if (this->IntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = this->mpu->getFIFOCount();

      this->mpu->getFIFOBytes(fifoBuffer, packetSize);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

      this->mpu->dmpGetQuaternion(&q, fifoBuffer);
      this->mpu->dmpGetGravity(&gravity, &q);
      this->mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
  } 
};
   