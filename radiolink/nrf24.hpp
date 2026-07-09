#pragma once
#include "../radiolink.hpp"
#include <SPI.h>
#include "RF24.h"
#include "../mess/rfmessage.hpp"

class NRF24 : public RadioLink {
protected:
  RF24 *radio = nullptr;
  uint8_t ce, csn, pipe = 1;
 
  
public:
  NRF24(uint8_t ce, uint8_t csn) : RadioLink() {
	this->ce = ce;
	this->csn = csn;
  }
  
  virtual ~NRF24() {
	delete radio;
  }
  
  bool setup() override {
	radio = new RF24(ce, csn);
	if (radio->begin()) {
		nbChannels = NB_CHANNELS;
		radio->setPALevel(RF24_PA_LOW);
		radio->setPayloadSize(sizeof(rfMess));
		radio->stopListening();
		radio->openReadingPipe(pipe, address);  // Pipe 1
		radio->startListening();
		return true;
	}
	else return false;
  }
  
  void update() override {
	if (radio->available(&pipe)) {              // is there a payload? get the pipe number that received it
      uint8_t bytes = radio->getPayloadSize();  // get the size of the payload
      radio->read(rfMess, bytes);             	// fetch payload from FIFO
    }
  }
  
  uint8_t address[6] = "1node";
};