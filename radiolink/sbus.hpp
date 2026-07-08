#pragma once
#include "../radiolink.hpp"
#include <sbus.h>

class Sbus : public RadioLink {
protected:
  bfs::SbusRx *sbusRx = nullptr;
  bfs::SbusData sbusData;
  HardwareSerial* hwSerial = nullptr;
  bool inverted = false;
  
public:
  Sbus(HardwareSerial* serial, uint8_t rxPin = -1, uint8_t txpin = -1, bool inv = false) : RadioLink(rxPin, txPin) {
	this->hwSerial = serial;
	this->inverted = inv;
  }
  
  virtual ~Sbus() {
	delete sbusRx;
	delete hwSerial;
  }
  
  bool setup() override {
	sbusRx = new bfs::SbusRx(hwSerial, rxPin, txPin, inverted); 
	sbusRx->Begin();
	nbChannels = (NB_CHANNELS < bfs::SbusData::NUM_CH) ? NB_CHANNELS : bfs::SbusData::NUM_CH;
	return true;
  }
  
  void update() override {
	if (sbusRx->Read()) {
	  sbusData = sbusRx->data();
	  for (int i=0; i<nbChannels; i++) {
		rfMess->channels[i] = sbusData.ch[i];
	  }
	}
  }
  
//  bool getData() {
//	if (sbusRx->Read()) {
//	  sbusData = sbusRx->data();
//	  for (int i=0; i<nbChannels; i++) {
//		rfMess->channels[i] = sbusData.ch[i];
//	  }
//	  return true;
//	}
//	return false;
//  }
};