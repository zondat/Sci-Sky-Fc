#pragma once
#include "../mcu.hpp"
#include "../config.h"
#include "nrf52_servo.hpp"

class NRF52 : public MCU {
protected:
	NRF52Servo* ppmGenerator[NB_PWM_CHN];
	int nbChn = 0;
public:
    NRF52() = default;
    virtual ~NRF52() = default;

	uint16_t readPpmChn(int chn) override {
		return ppmGenerator[chn]->getCurrentValue();
	}

	int registerPpmChn(int pin) override {
		NRF52Servo* servo = new NRF52Servo();
		if (servo->init(pin)) {
			ppmGenerator[nbChn++] = servo;
			return nbChn-1;
		}
		return -1;
	}
	
	void generatePpm(int chn, uint16_t value) override {
		ppmGenerator[chn]->writeMicroseconds(value);
	}
};
