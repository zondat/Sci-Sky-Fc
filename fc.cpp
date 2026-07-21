#include "fc.hpp"

// #include "mcu/esp32.hpp"
#include "mcu/nrf52.hpp"

#if defined(ARDUINO_ARCH_NRF52)
  #include <FreeRTOS.h>
  #include <task.h>
  #include <semphr.h>
#endif

// ========== SEMAPHORE ========== //
SemaphoreHandle_t semControlFlight = xSemaphoreCreateBinary();

Fc::Fc() {
  mcu = new NRF52(); 
  // mcu = new Esp32();
  imu = new MPU6500(PIN_MISO, PIN_MOSI, PIN_SCK, PIN_SS, -1);

  // Serial1.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);
  // rx = new CRSF(&Serial1);
  // rx = new RFLite(PIN_SCK, PIN_MOSI, PIN_MISO, 0, 1, 0, true);
  rx = new Nrf2Nrf();
  
  armed = new Armed(this);
  disarmed = new Disarmed(this);
  currentState = disarmed;

  altEstimator = new Brute();
  mode = new PitchRollStabilizer(this);

  motor = new StandardPwm(mcu, PIN_THR);
  servoPitch = new AServo(mcu, PIN_PITCH);
  servoAileron_1 = new AServo(mcu, PIN_AIL_1);
  servoAileron_2 = new AServo(mcu, PIN_AIL_2);
}

Fc::~Fc() {
  delete rx;
  delete imu;
  delete mcu;
  delete mode;
  delete motor;
  delete servoPitch;
  delete servoAileron_1;
  delete servoAileron_2;
  delete armed;
  delete disarmed;
  delete altEstimator;
}

bool Fc::setup() {
  if (!mcu || !imu || !altEstimator || !rx) return false;

  if (!rx->setup()) return false;
  if (motor) motor->setup();
  if (servoPitch) servoPitch->setup();
  if (servoAileron_1) servoAileron_1->setup();
  if (servoAileron_2) servoAileron_2->setup();
  if (mode) mode->setup();
  if (!mcu->setup()) return false;
  if (!imu->setup()) return false;
  if (!altEstimator->setup()) return false;

  delay(500);
  return true;
}

void Fc::update() {
  imu->update();
  altEstimator->update();
  rx->update();
}

void Fc::calib() {
  imu->calib();
  altEstimator->calib();
  rx->calib();
}

void Fc::imuUpdate() {
  imu->update();

  // Check to trigger control flight
  uint32_t currentTimeUs = micros();
  uint32_t elapsedTimeUs = currentTimeUs - timeTable.getLastWakeupUs();
  if (elapsedTimeUs > CONTROL_INTERVAL_US - TRIGGER_JITTER_US
      && elapsedTimeUs < CONTROL_INTERVAL_US + TRIGGER_JITTER_US) {
    timeTable.setLastWakeupUs(currentTimeUs);
    timeTable.setElapsedTimeUs(elapsedTimeUs);
    xSemaphoreGive(semControlFlight);  // trigger control flight task
  }
}

void Fc::rxUpdate() {
  rx->update();

  // Arming
  if (rx->getChannel(CHN_ARM) > 1200) setStateArmed(); 
  else setStateDisarmed();

  //Update new signal
  if (DEBUG) Serial.println("Chn1: " + String( rx->getChannel(1)) + ", Chn2: " + String( rx->getChannel(2)) + ", Chn3: " + String( rx->getChannel(3)));

  // Trigger control
  uint32_t currentTimeUs = micros();
  timeTable.setElapsedTimeUs(currentTimeUs - timeTable.getLastWakeupUs());
  timeTable.setLastWakeupUs(currentTimeUs);
  xSemaphoreGive(semControlFlight);  // Trigger task ControlFlight()
}

void Fc::controlFlight() {
  if (xSemaphoreTake(semControlFlight, portMAX_DELAY) == pdTRUE) {
    mode->computeCmd(&cmdRoll, &cmdPitch, &cmdYaw,
                     rx->getChannel(CHN_AILERON),
                     rx->getChannel(CHN_ELEVATOR),
                     rx->getChannel(CHN_RUDDER),
                     ((float)(timeTable.getElapsedTimeUs())) * 1e-6);

    // Serial.println("CmdRoll: " + String(cmdRoll));
    // delay(100);

    setThrust(rx->getChannel(CHN_THRUST));
    servoAileron_1->setValue(cmdRoll);
    servoAileron_2->setValue(cmdRoll);
    servoPitch->setValue(cmdPitch);
  }
}

// void Fc::createRxIbus(HardwareSerial* hwSerial) {
//   rx = new Ibus(hwSerial);
// }

void Fc::createRxNrf2Nrf() {
  #if defined(ARDUINO_ARCH_NRF52)
    rx = new Nrf2Nrf();    
  #endif  
}

void Fc::createRxNRF24(uint8_t ce, uint8_t csn) {
  rx = new NRF24(ce, csn);
}

void Fc::createRxNRFLite(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t ce, uint8_t csn, uint8_t radioId, bool alreadyInitSPI) {
  #if !defined(ARDUINO_ARCH_NRF52)
    rx = new RFLite(sck, mosi, miso, ce, csn, radioId, alreadyInitSPI);    
  #endif  
}

void Fc::createRxCrsf(HardwareSerial* hwSerial) {
  #if !defined(ARDUINO_ARCH_NRF52)
    rx = new CRSF(hwSerial);    
  #endif  
}

void Fc::createRxSbus(HardwareSerial* hwSerial) {
  #if !defined(NRF52840_XXAA)
    rx = new Sbus(hwSerial);    
  #endif    
}

void Fc::createMpu6050(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin) {
  imu = new MPU6050(miso, mosi, sck, cs, irPin);
}

void Fc::createMpu6050(TwoWire* const w, uint8_t const addr, uint8_t irPin) {
  imu = new MPU6050(w, addr, irPin);
}

void Fc::createMpu6500(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin) {
  imu = new MPU6500(miso, mosi, sck, cs, irPin);
}

void Fc::createMpu6500(TwoWire* const w, uint8_t const addr, uint8_t irPin) {
  imu = new MPU6500(w, addr, irPin);
}

void Fc::createMpu9250(uint8_t miso, uint8_t mosi, uint8_t sck, uint8_t cs, uint8_t irPin) {
  imu = new MPU9250(miso, mosi, sck, cs, irPin);
}

void Fc::createMpu9250(TwoWire* const w, uint8_t const addr, uint8_t irPin) {
  imu = new MPU9250(w, addr, irPin);
}

// void Fc::createMcuEsp32() { mcu = new Esp32();}
// void Fc::createMcuStm32() { mcu = new Stm32();}
// void Fc::createMcuNrf52() { mcu = new NRF52();}