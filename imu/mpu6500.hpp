#pragma once
#include "../imu.hpp"
#include <MPU6500_WE.h>
#include <Wire.h>

class MPU6500 : public IMU {
private:
  TwoWire* wireI2c = nullptr;
  uint8_t addr = 0x68;
  uint8_t miso = -1, mosi = -1, sck = -1, cs = -1;
  uint8_t interruptPin = -1;
  
protected:
	MPU6500_WE *imu = nullptr;
public:
  MPU6500(TwoWire * const w = &Wire, uint8_t const addr = 0x68, uint8_t irPin = -1) : IMU()  {
	useSpi = false;
	wireI2c = w;
	this->addr = addr;
	this->interruptPin = irPin;
  }
  
  MPU6500(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin = -1) : IMU()  {
	useSpi = true;
	this->miso = miso;
	this->mosi = mosi;
	this->sck  = sck;
	this->cs   = cs;
	this->interruptPin = irPin;
  }
  
  virtual ~MPU6500() {
	  delete imu;
  }
  
  bool setup() override {
	if (useSpi) imu = new MPU6500_WE(&SPI, cs, mosi, miso, sck, true);
	else imu = new MPU6500_WE(wireI2c, addr);
	
	if (imu->init()) {
	  imu->enableGyrDLPF();
	  imu->setGyrDLPF(MPU6500_DLPF_6);
	  imu->setSampleRateDivider(5);
	  imu->setGyrRange(MPU6500_GYRO_RANGE_500);
	  imu->setAccRange(MPU6500_ACC_RANGE_16G);
	  imu->enableAccDLPF(true);
	  imu->setAccDLPF(MPU6500_DLPF_6);
	  
	  if (interruptPin != -1) {
		// Enable interrupt 
		imu->enableIntLatch(true);
		imu->enableInterrupt(MPU6500_DATA_READY); 
	  
		// Setup mail-box communication
		imu->setFifoMode(MPU6500_CONTINUOUS);
		imu->startFifo(MPU6500_FIFO_ACC_GYR);
	  }
	  
	  ahr->setup();
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
	
	data->mx = -1;
	data->my = -1;
	data->mz = -1;
	
	ahr->update();
  }
};
