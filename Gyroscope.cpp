#include <Gyroscope.h>


Gyro::Gyro(gyro_config_t cfg) {
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