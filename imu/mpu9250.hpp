#pragma once
#include "../imu.hpp"
#include <MPU9250_WE.h>
#include <Wire.h>

class MPU9250 : public IMU {
private:
  TwoWire* wireI2c = nullptr;
  uint8_t addr = 0x68;
  uint8_t miso = -1, mosi = -1, sck = -1, cs = -1;
  uint8_t interruptPin = -1;
  
protected:
	MPU9250_WE *imu = nullptr;
	
public:
  MPU9250(TwoWire * const w = &Wire, uint8_t const addr = 0x68, uint8_t irPin = -1) : IMU()  {
	useSpi = false;
	wireI2c = w;
	this->addr = addr;
	this->data = new Imu9dMessage();
	this->interruptPin = irPin;
  }
  
  MPU9250(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin = -1) : IMU()  {
	useSpi = true;
	this->miso = miso;
	this->mosi = mosi;
	this->sck  = sck;
	this->cs   = cs;
	this->data = new Imu9dMessage();
	this->interruptPin = irPin;
  }
  
  virtual ~MPU9250() {
	  delete imu;
  }
  
  bool setup() override {
	if (useSpi) imu = new MPU9250_WE(&SPI, cs, true);
	else imu = new MPU9250_WE(wireI2c, addr);
	
	if (imu->init()) {
	  imu->enableGyrDLPF();
	  imu->setGyrDLPF(MPU9250_DLPF_6);
	  imu->setSampleRateDivider(5);
	  imu->setGyrRange(MPU9250_GYRO_RANGE_500);
	  imu->setAccRange(MPU9250_ACC_RANGE_16G);
	  imu->enableAccDLPF(true);
	  imu->setAccDLPF(MPU9250_DLPF_6);
	  
	  if (interruptPin != -1) {
		// Enable interrupt 
		imu->enableIntLatch(true);
		imu->enableInterrupt(MPU9250_DATA_READY); 
	  
		// Setup mail-box communication
		imu->setFifoMode(MPU9250_CONTINUOUS);
		imu->startFifo(MPU9250_FIFO_ACC_GYR);
	  }
	  return true;
	}
	return false;
  }
  
  void calib() override {
	delay(1000);
	imu->autoOffsets();
  } 
  
  void update() override {
	xyzFloat gValue = imu->getGValues();
	data->ax = gValue.x;
	data->ay = gValue.y;
	data->az = gValue.z;
	
	xyzFloat gyr = imu->getGyrValues();
	data->gx = gyr.x;
	data->gy = gyr.y;
	data->gz = gyr.z;
	
	xyzFloat mag = imu->getMagValues();
	data->mx = mag.x;
	data->my = mag.y;
	data->mz = mag.z;

  }
  
};