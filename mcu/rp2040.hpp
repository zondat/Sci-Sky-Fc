#pragma once
#include "../mcu.hpp"
#include <Servo.h>

class Rp2040 : public MCU {
protected:
	Servo* ppmGenerator[NB_PWM_CHN];
	int nbChn = 0;
public:
    Rp2040() = default;
    virtual ~Rp2040() = default;

	uint16_t readPpmChn(int chn) override {
		return ppmGenerator[chn]->readMicroseconds();
	}

	int registerPpmChn(int pin) override {
		Servo* servo = new Servo();
		servo->attach(pin);
		ppmGenerator[nbChn++] = servo;
		return nbChn-1;
	}
	
	void generatePpm(int chn, uint16_t value) override {
		ppmGenerator[chn]->writeMicroseconds(value);
	}
};