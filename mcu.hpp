#pragma once
#include "component.hpp"

class MCU : public Component {
public:
	MCU() = default;
	virtual ~MCU() = default;

	virtual bool setup() override {
		return true;
	}
	virtual void update() override {}
	virtual void calib() override {}

	virtual void generatePwm(int pin, uint16_t value) {
		analogWrite(pin, value);
	}

	uint8_t mosi=-1, miso=-1, sck=-1, ss=-1;
	uint8_t rx=-1, tx=-1;

	virtual uint16_t readPpmChn(int pin) = 0;
	virtual int registerPpmChn(int pin) = 0;
	virtual void generatePpm(int chn, uint16_t value) = 0;
};