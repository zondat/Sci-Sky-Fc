#pragma once
#include "sensor.hpp"
#include "mess/rfmessage.hpp"

class RadioLink : public Sensor {
protected:
  RfMessage *rfMess = nullptr;
  uint8_t nbChannels = -1;
  uint8_t txPin = -1, rxPin = -1;

public:
  RadioLink() {
    rfMess = new RfMessage();
  }

  RadioLink(uint8_t txPin, uint8_t rxPin) : RadioLink() {
    this->txPin = txPin;
    this->rxPin = rxPin;
  }

  virtual ~RadioLink() {
    delete rfMess;
  }

  uint16_t getChannel(uint8_t idx) {
    return rfMess->channels[idx];
  }

  RfMessage* getMessage() {
    return rfMess;
  }

  void calib() override {}
  void update() override {}
  uint8_t getNbChannels() { return nbChannels;}
};