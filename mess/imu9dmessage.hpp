#pragma once
#include "imu6dmessage.hpp"

class Imu9dMessage : public Imu6dMessage {
public:
  float mx, my, mz;
};