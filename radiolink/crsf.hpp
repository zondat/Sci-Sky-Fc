#pragma once
#include "../radiolink.hpp"
#include "CRSFforArduino.hpp"

class CRSF : public RadioLink {
protected:
  CRSFforArduino *crsf = nullptr;
  HardwareSerial* hwSerial = nullptr;
  
public:
  CRSF(HardwareSerial* serial) : RadioLink() {
	this->hwSerial = serial;
  }
  
  virtual ~CRSF() {
	delete crsf;
	delete hwSerial;
  }
  
  bool setup() override {
	crsf = new CRSFforArduino(hwSerial);
	if (crsf->begin()) {
		nbChannels = (NB_CHANNELS < crsfProtocol::RC_CHANNEL_COUNT) ? NB_CHANNELS : (crsfProtocol::RC_CHANNEL_COUNT);
		return true;
	}
	else return false;
  }
  
  void update() override {
	crsf->update();
	for (int i=0; i<nbChannels; i++) {
	  rfMess->channels[i] = crsf->rcToUs(crsf->getChannel(i));
	}
  }
};