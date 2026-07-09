#pragma once
#include "../config.h"
#include "../radiolink.hpp"
#include <IBusBM.h>

class Ibus : public RadioLink {
protected:
  IBusBM *ibus = nullptr;
  HardwareSerial* hwSerial = nullptr;
  
public:
  Ibus(HardwareSerial* serial) : RadioLink() {
	this->hwSerial = serial;
  }
  
  virtual ~Ibus() {
	delete ibus;
	delete hwSerial;
  }
  
  bool setup() override {
	ibus = new IBusBM();
	ibus->begin(*hwSerial);
	nbChannels = (NB_CHANNELS < IBUS_PROTOCOL_CHANNELS) ? NB_CHANNELS : IBUS_PROTOCOL_CHANNELS;
	return true;
  }
  
  void update() override {
	for (int i=0; i<nbChannels; i++) {
	  rfMess->channels[i] = ibus->readChannel(i);
	}
  }
  
};