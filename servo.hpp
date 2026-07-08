#pragma once
#include "actuator.hpp"

class AServo : public Actuator {
private:
	int chn = -1;
public:
	AServo(MCU* mcu, int pin, uint16_t freq = 50, uint16_t minValue = 1000, uint16_t maxValue = 2000) : Actuator(mcu, pin, freq, minValue, maxValue) {}
	~AServo() = default;

	bool setup() override {
		chn = mcu->registerPpmChn(pin);
		return chn!=-1;
	}

	void calib() override {
		setValue(1500);
	}

	void update() override {}
	
	void setValue(uint16_t us) override { 
		mcu->generatePpm(chn, us);
	}
	
	uint16_t getValue() override {
		return mcu->readPpmChn(chn);
	}
};