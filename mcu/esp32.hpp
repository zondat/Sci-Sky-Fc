#pragma once
#include "../mcu.hpp"
#include "config.h"
#include <ESP32Servo.h>

class Esp32 : public MCU {
protected:
	Servo* ppmGenerator[NB_PWM_CHN];
	int nbChn = 0;
public:
    Esp32() = default;
    virtual ~Esp32() = default;

	bool setup() override {
		ESP32PWM::allocateTimer(0);
		ESP32PWM::allocateTimer(1);
		ESP32PWM::allocateTimer(2);
		ESP32PWM::allocateTimer(3);
		return true;
	}

	uint16_t readPpmChn(int chn) override {
		return ppmGenerator[chn]->readMicroseconds();
	}

	int registerPpmChn(int pin) override {
		Servo* servo = new Servo();
		servo->setPeriodHertz(50);    // standard 50 hz servo
		servo->attach(pin, 1000, 2000);
		ppmGenerator[nbChn++] = servo;
		return nbChn-1;
	}
	
	void generatePpm(int chn, uint16_t value) override {
		ppmGenerator[chn]->writeMicroseconds(value);
	}
};