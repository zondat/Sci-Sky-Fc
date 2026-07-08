#pragma once
#include "../mcu.hpp"
#include "Adafruit_TinyUSB.h"
#include "nrf52_servo.hpp"

class NRF52 : public MCU {
protected:
	NRF52_Servo* ppmGenerator[NB_PWM_CHN];
	int nbChn = 0;
public:
    NRF52() = default;
    virtual ~NRF52() = default;

	uint16_t readPpmChn(int chn) override {
		return ppmGenerator[chn]->get_current_value();
	}

	int registerPpmChn(int pin) override {
		NRF52_Servo* servo = new NRF52_Servo();
		servo->init(pin);
		ppmGenerator[nbChn++] = servo;
		return nbChn-1;
	}
	
	void generatePpm(int chn, uint16_t value) override {
		ppmGenerator[chn]->writeMicroseconds(value);
	}
};