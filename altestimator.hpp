#pragma once

#include "sensor.hpp"

class AltEstimator : public Sensor {
public:
  AltEstimator() = default;
  virtual ~AltEstimator() = default;
  virtual float estimateAlt() = 0;
  virtual float estimateClimbSpeed() = 0;
  
  bool setup() override {return true;}
  void update() override {}
  void calib() override {}
};