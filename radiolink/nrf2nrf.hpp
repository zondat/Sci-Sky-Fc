#pragma once
#include "../radiolink.hpp"
#include <SPI.h>
#include "nrf_to_nrf.h"
#include "../mess/rfmessage.hpp"

class Nrf2Nrf : public RadioLink {
protected:
  nrf_to_nrf *radio = nullptr;
  uint8_t pipe = 1;
  uint8_t address[6] = {"1Node"};
 
public:
  Nrf2Nrf() : RadioLink() {}
  
  virtual ~Nrf2Nrf() {
	delete radio;
  }
  
  bool setup() override {
	radio = new nrf_to_nrf();
	if (radio->begin()) {
		nbChannels = NB_CHANNELS;
		radio->setPALevel(NRF_PA_LOW);
		radio->setPayloadSize(sizeof(RfMessage));
		radio->openReadingPipe(1, address);  // Pipe 1
		radio->startListening();
		return true;
	}
	else return false;
  }
  
  void update() override {
	while (radio->available(&pipe)) {              // is there a payload? get the pipe number that received it
      uint8_t bytes = radio->getPayloadSize();  // get the size of the payload
      radio->read(rfMess, bytes);             	// fetch payload from FIFO
    }
  }
  
  
};