#pragma once

#include "../altestimator.hpp"

class Brute : public AltEstimator {
public:
  Brute() = default;
  virtual ~Brute() = default;

  float estimateAlt() override {
	return 0;
  }
  
  float estimateClimbSpeed() override {
	return 0;
  }
};