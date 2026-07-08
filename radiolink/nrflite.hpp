#pragma once
#include "../radiolink.hpp"
#include <SPI.h>
#include <NRFLite.h>
#include "../mess/rfmessage.hpp"

class RFLite : public RadioLink {
protected:
  NRFLite *radio = nullptr;
  uint8_t mosi, miso, sck, ce, csn;
  RfMessage *rfMess = nullptr;
  bool alreadyInitSPI = true;
  uint8_t radioId = 0;
  
public:
  RFLite(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t ce, uint8_t csn, RfMessage *rfMess, uint8_t id, bool alreadyInitSPI = false) : RadioLink() {
	this->ce = ce;
	this->csn = csn;
	this->sck = sck;
	this->miso = miso;
	this->mosi = mosi;
	this->radioId = id;
	this->rfMess = rfMess;
	this->alreadyInitSPI = alreadyInitSPI;
  }
  
  virtual ~RFLite() {
	delete radio;
  }
  
  bool setup() override {
	radio = new NRFLite();
	if (!alreadyInitSPI) SPI.begin(sck, miso, mosi, csn);
	return radio->init(radioId, ce, csn, NRFLite::BITRATE2MBPS, 100, !alreadyInitSPI);
  }
  
  void update() override {
	while (radio->hasData()) {              
      radio->readData(rfMess);         		
    }
  }
};