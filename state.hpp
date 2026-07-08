#pragma once
#include "config.h"

class Fc;
class State {
protected:
	Fc* fc = nullptr;
public:
  State(Fc *fc);
	virtual void setThrust(uint16_t value) = 0;
	virtual void arm() = 0;
	virtual void disarm() = 0;
};

class Armed : public State {
public:
	Armed(Fc *fc);
	virtual ~Armed();
	
	void setThrust(uint16_t value) override;
	void arm() override {}
	void disarm() override;
};

class Disarmed : public State {
public:
	Disarmed(Fc *fc);
	virtual ~Disarmed();
	
	void setThrust(uint16_t value) override;
	void arm() override;
	void disarm() override { }
};