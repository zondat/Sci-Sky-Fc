#pragma once

//#include <Arduino.h>
#include "../motor.hpp"

class PwmBrushed : public Motor {
private:
	int current;
public:
    PwmBrushed(MCU* mcu, int pin, uint16_t freq=50, uint16_t minUs=0, uint16_t maxUs=255) : Motor(mcu, pin, freq, minUs, maxUs){}
    ~PwmBrushed() override = default;

    bool setup() override {
		pinMode(pin, OUTPUT);
		return true;
	}
	
	void calib() override {
		setValue(minValue);
		current = minValue;
	}
	
	void setValue(uint16_t val) override {
		mcu->generatePwm(pin, val);
		current = val;
	}
	
	uint16_t getValue() override {
		return current;
	}
};

class PpmBrushed : public Motor {
private:
	int chn = -1;
public:
    PpmBrushed(MCU* mcu, int pin, uint16_t freq, uint16_t minUs, uint16_t maxUs) : Motor(mcu, freq, minUs, maxUs, pin){}
    ~PpmBrushed() = default;

    bool setup() override {
		chn = mcu->registerPpmChn(pin);
		return chn!=-1;
	}
	
	void calib() override {
		setValue(1500);
	}
	
	void setValue(uint16_t val) override {
		mcu->generatePpm(chn, val);
	}
	
	uint16_t getValue() override {
		return mcu->readPpmChn(chn);
	}
};