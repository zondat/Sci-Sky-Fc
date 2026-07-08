#pragma once

#include "component.hpp"
#include "mcu.hpp"

class Actuator : public Component {
protected:
  int pin = -1;
  uint16_t maxValue, minValue;
  uint16_t frequency;
  MCU* mcu = nullptr;
  
public:
  Actuator(MCU* mcu, int pin, uint16_t freq = 50, uint16_t minValue = 1000, uint16_t maxValue = 2000) {
    this->mcu = mcu;
    this->pin = pin;
    this->frequency = freq;
    this->minValue = minValue;
    this->maxValue = maxValue;
  }
  virtual ~Actuator() = default;

  virtual void update() override {}
  virtual void calib() override {}

  void setPin(int p) {
    pin = p;
  }
  int getPin() const {
    return pin;
  }

  virtual void setValue(uint16_t v) = 0;
  virtual uint16_t getValue() = 0;
};