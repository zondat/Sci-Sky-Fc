#pragma once

#include "config.h"
#include "mode.hpp"

#include "radiolink/crsf.hpp"
// #include "radiolink/ibus.hpp"
#include "radiolink/sbus.hpp"
#include "radiolink/nrflite.hpp"
#include "radiolink/nrf24.hpp"
 
#include "servo.hpp"
#include "esc/brushed.hpp"
#include "esc/standard.hpp"

#include "imu.hpp"
#include "imu/mpu6050.hpp"
#include "imu/mpu6500.hpp"
#include "imu/mpu9250.hpp"

#include "radiolink.hpp"
#include "alt/brute.hpp"

#include "state.hpp"

#include "mcu.hpp"
#include "mcu/esp32.hpp"

#include "timetable.hpp"

class Fc : public Component {
protected:
  IMU* imu = nullptr;
  MCU* mcu = nullptr;
  Mode* mode = nullptr;
  RadioLink* rx = nullptr;
  AltEstimator* altEstimator = nullptr;
  Actuator* motor = nullptr, *servoAileron_1 = nullptr, *servoAileron_2 = nullptr, *servoPitch = nullptr;
  State *armed = nullptr, *disarmed = nullptr, *currentState = nullptr;
  RfMessage* currentRxMess;

  volatile TimeTable timeTable;
  uint16_t cmdRoll = 0, cmdPitch = 0, cmdYaw = 0; 

public:
  Fc();
  virtual ~Fc();

  bool setup() override ;
  void update() override;
  void calib() override;

  void rxUpdate();
  void imuUpdate();
  void controlFlight();

  // void createRxIbus(HardwareSerial* hwSerial);
  void createRxCrsf(HardwareSerial* hwSerial);
  void createRxSbus(HardwareSerial* hwSerial);
  void createRxNRF24(uint8_t ce, uint8_t csn, RfMessage *rfMess);
  void createRxNRFLite(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t ce, uint8_t csn, RfMessage *rfMess, uint8_t RADIO_ID, bool alreadyInitSPI = false);

  void createBruteAltEstimator() {altEstimator = new Brute();}

  // void createMcuEsp32();
  // void createMcuStm32();
  // void createMcuNrf52();

  void createMpu6050(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin = -1);
  void createMpu6050(TwoWire* const w = &Wire, uint8_t const addr = 0x68, uint8_t irPin = -1);

  void createMpu6500(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin = -1);
  void createMpu6500(TwoWire* const w = &Wire, uint8_t const addr = 0x68, uint8_t irPin = -1);

  void createMpu9250(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin = -1);
  void createMpu9250(TwoWire* const w = &Wire, uint8_t const addr = 0x68, uint8_t irPin = -1);

  void createStandardMotor(int freq=50, int minUs=1000, int maxUs=2000, int pin=-1)   { motor = new StandardPwm(mcu, freq, minUs, maxUs, pin); }
  void createPpmBrushedMotor(int freq=50, int minUs=800, int maxUs=2000, int pin=-1)  { motor = new PpmBrushed(mcu, freq, minUs, maxUs, pin); }
  void createPwmBrushedMotor(int freq=50, int minUs=0, int maxUs=255, int pin=-1)     { motor = new PwmBrushed(mcu, freq, minUs, maxUs, pin); }

  Actuator* getMotor() { return motor; }

  float getAhrRoll() {return imu->getAhr()->roll; }
  float getAhrPitch() {return imu->getAhr()->pitch; }
  float getAhrYaw() {return imu->getAhr()->yaw; }
  float getAhrGx() {return imu->getAhr()->cgx; }
  float getAhrGy() {return imu->getAhr()->cgy; }
  float getAhrGz() {return imu->getAhr()->cgz; }

  void setThrust(uint16_t value) { 
    currentState->setThrust(value); 
  }

  void setStateDisarmed() {
    currentState = disarmed;
  }
  void setStateArmed() {
    currentState = armed;
  }
};