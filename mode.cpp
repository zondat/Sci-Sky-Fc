#include "mode.hpp"
#include "fc.hpp"
#include "config.h"

// =======================================================
Mode::Mode(Fc *fc) {
  this->fc = fc;
}
Mode::~Mode() {
  delete pidRoll;
  delete pidPitch;
  delete pidYaw;
}

bool Mode::setup() {
  pidRoll = new PIFF(Kp_roll, Ki_roll, Kd_roll, Kf_roll, -MAX_ROLL, MAX_ROLL, -I_LIMIT, I_LIMIT); 
  pidPitch = new PIFF(Kp_pitch, Ki_pitch, Kd_pitch, Kf_pitch,-MAX_PITCH, MAX_PITCH, -I_LIMIT, I_LIMIT); 
  pidYaw = new PIFF(0, 0, 0, 0, 0, 0, 0); 
  return true;
}

// =======================================================
PitchRollStabilizer::PitchRollStabilizer(Fc *fc): Mode(fc) {}
PitchRollStabilizer::~PitchRollStabilizer() {}

void PitchRollStabilizer::computeCmd(uint16_t *cmdRoll, uint16_t *cmdPitch, uint16_t *cmdYaw, uint16_t rlRoll, uint16_t rlPitch, uint16_t rlYaw, float dt) {

  float desRoll = map(rlRoll, PPM_MIN, PPM_MAX, -MAX_ROLL, MAX_ROLL);             // Setpoint in degree
  float ahrRoll = fc->getAhrRoll();
  float derivativeRoll = fc->getAhrGx();
  float cmdRollInDegree = pidRoll->compute(desRoll, ahrRoll, dt, derivativeRoll);
  *cmdRoll = map(cmdRollInDegree, -MAX_ROLL, +MAX_ROLL, PPM_MIN, PPM_MAX);  // Re-scale from degree to ppm signal
 
  float desPitch = map(rlPitch, PPM_MIN, PPM_MAX, -MAX_PITCH, MAX_PITCH);         // Setpoint in degree
  float ahrPitch = fc->getAhrPitch();
  float derivativePitch = fc->getAhrGy();
  float cmdPitchInDegree = pidPitch->compute(desPitch, ahrPitch, dt, derivativePitch);
  *cmdPitch = map(cmdPitchInDegree, -MAX_ROLL, +MAX_ROLL, PPM_MIN, PPM_MAX);  // Re-scale from degree to ppm signal

  //Yaw PID - passthru rcin
  *cmdYaw = rlYaw;
  //  (void) integral_yaw;
  //  (void) error_yaw_prev;
}

// =========================================================
Manual::Manual(Fc *fc) : Mode(fc) {}
Manual::~Manual() {}

void Manual::computeCmd(uint16_t *cmdRoll, uint16_t *cmdPitch, uint16_t *cmdYaw, uint16_t rlRoll, uint16_t rlPitch, uint16_t rlYaw, float dt) {
  *cmdRoll = rlRoll;
  *cmdPitch = rlPitch;
  *cmdYaw = rlYaw;
}

// ==========================================================
RollStabilizer::RollStabilizer(Fc *fc)  : Mode(fc) {}
RollStabilizer::~RollStabilizer() {}

void RollStabilizer::computeCmd(uint16_t *cmdRoll, uint16_t *cmdPitch, uint16_t *cmdYaw, uint16_t rlRoll, uint16_t rlPitch, uint16_t rlYaw, float dt) {
  float desRoll = map(rlRoll, PPM_MIN, PPM_MAX, -MAX_ROLL, MAX_ROLL);             // Setpoint in degree
  float ahrRoll = fc->getAhrRoll();
  float derivativeRoll = fc->getAhrGx();
  float cmdRollInDegree = pidRoll->compute(desRoll, ahrRoll, dt, derivativeRoll);
  *cmdRoll = map(cmdRollInDegree, -MAX_ROLL, +MAX_ROLL, PPM_MIN, PPM_MAX);  // Re-scale from degree to ppm signal

  //Pitch PID - passthru rcin
  *cmdPitch = rlPitch;

  //Yaw PID - passthru rcin
  *cmdYaw = rlYaw;
}
