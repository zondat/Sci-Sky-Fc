#pragma once
#include "actuator.hpp"

class Motor : public Actuator {
public:
	Motor(MCU* mcu, int p, uint16_t freq = 50, uint16_t minValue = 800, uint16_t maxValue = 2000)
	  : Actuator(mcu, p, freq, minValue, maxValue) {}
	virtual ~Motor() = default;
	
	void stop() {
		setValue(minValue);
	}
};