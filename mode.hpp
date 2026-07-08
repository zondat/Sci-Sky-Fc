#pragma once 

#include "config.h"
#include "pid/piff.hpp"

class Fc;

class Mode {
protected:
  Fc *fc = nullptr;
  PID *pidRoll = nullptr, *pidPitch = nullptr, *pidYaw = nullptr;

public:
  Mode(Fc *fc);
  virtual ~Mode();
  bool setup();
  virtual void computeCmd(uint16_t *cmdRoll, uint16_t *cmdPitch, uint16_t *cmdYaw, uint16_t rlRoll, uint16_t rlPitch, uint16_t rlYaw, float dt) = 0;
  float integralRoll = 0;
  float integralPitch = 0;
  float integralYaw = 0;
  float errorYawPrev = 0;
};

class PitchRollStabilizer : public Mode {
public:
  PitchRollStabilizer(Fc* fc);
  virtual ~PitchRollStabilizer();
  void computeCmd(uint16_t *cmdRoll, uint16_t *cmdPitch, uint16_t *cmdYaw, uint16_t rlRoll, uint16_t rlPitch, uint16_t rlYaw, float dt) override;
};

class Manual : public Mode {
public:
  Manual(Fc* fc);
  virtual ~Manual();
  void computeCmd(uint16_t *cmdRoll, uint16_t *cmdPitch, uint16_t *cmdYaw, uint16_t rlRoll, uint16_t rlPitch, uint16_t rlYaw, float dt) override;
};

class RollStabilizer : public Mode {
public:
  RollStabilizer(Fc* fc);
  virtual ~RollStabilizer();
  void computeCmd(uint16_t *cmdRoll, uint16_t *cmdPitch, uint16_t *cmdYaw, uint16_t rlRoll, uint16_t rlPitch, uint16_t rlYaw, float dt) override;
};

// #endif