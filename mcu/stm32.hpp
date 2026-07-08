#pragma once
#include "../mcu.hpp"
#include <Servo.h>

class Stm32 : public MCU {
protected:
	Servo* ppmGenerator[NB_PWM_CHN];
	int nbChn = 0;
public:
    Stm32() = default;
    virtual ~Stm32() = default;

	int readPpmChn(int chn) override {
		return ppmGenerator[chn]->readMicroseconds();
	}

	int registerPpmChn(int pin) override {
		Servo* servo = new Servo();
		servo->attach(pin);
		ppmGenerator[nbChn++] = servo;
		return nbChn-1;
	}
	
	void generatePpm(int chn, int value) override {
		ppmGenerator[chn]->writeMicroseconds(value);
	}
};