#include "state.hpp"
#include "fc.hpp"

State::State(Fc *fc) {this->fc = fc;}

Disarmed::Disarmed(Fc *fc) : State(fc) {}
Disarmed::~Disarmed() {}

void Disarmed::setThrust(uint16_t value) {}
void Disarmed::arm() { fc->setStateDisarmed(); }

Armed::Armed(Fc *fc) : State(fc) {}
Armed::~Armed() {}

void Armed::setThrust(uint16_t value) { 
	fc->getMotor()->setValue(value);
}

void Armed::disarm() { fc->setStateDisarmed(); }