#pragma once

#include "../motor.hpp"

class StandardPwm : public Motor {
private:
	int chn = -1;
public:
    StandardPwm(MCU* mcu, int pin, uint16_t freq = 50, uint16_t minUs = 800, uint16_t maxUs=2000) : Motor(mcu, pin, freq, minUs, maxUs){}
    ~StandardPwm() = default;

    bool setup() override {
		chn = mcu->registerPpmChn(pin);
		return chn!=-1;
	}
	
	void calib() override {
		for (int ppm=maxValue; ppm>=minValue; ppm-=25) {
			setValue(ppm);
			delay(10);
		}
	}
	
	void setValue(uint16_t val) override {
		mcu->generatePpm(chn, val);
	}
	
	uint16_t getValue() override {
		return mcu->readPpmChn(chn);
	}
};