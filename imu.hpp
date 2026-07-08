#pragma once

#include "sensor.hpp"
#include "mess/imu9dmessage.hpp"
#include "ahr.hpp"
#include <SPI.h>

class IMU : public Sensor {
protected:
  bool useSpi = true;
  Imu9dMessage* data = nullptr;
  Ahr* ahr = nullptr;

public:
  IMU() {
    data = new Imu9dMessage();
    ahr = new Ahr(data);
  }

  virtual ~IMU() {
    delete data;
    delete ahr;
  }

  Imu9dMessage* getData() {
    update();
    return data;
  }
  
  Ahr* getAhr() { return ahr; }
};