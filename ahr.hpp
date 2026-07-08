#pragma once

#include "config.h"
#include "component.hpp"
#include "mess/imu9dmessage.hpp"
#include "mess/quaternion.hpp"
#include "ahr/filter.hpp"

class Ahr : public Component {
public:
  Ahr(Imu9dMessage* imuData) {
    this->imuData = imuData;
  }
  virtual ~Ahr() {delete filter;}
  
  bool setup() override;
  void update() override;
  void calib() override {}
  
  void init();
  void updateEulerAngles();
  float getAccelUp(); 			//get acceleration in earth-frame up direction in [m/s^2]  

  float roll = 0;               // roll in degrees: -180 to 180, roll right is positive
  float pitch = 0;              // pitch in degrees: -90 to 90, pitch up is positive
  float yaw = 0;                // yaw in degrees: -180 to 180, yaw right is positive

  float cgx = 0, cgy = 0, cgz = 0; // corrected and filtered imu gyro measurements in [deg/sec]
  float cax = 0, cay = 0, caz = 0; // corrected and filtered imu accel measurements in [g]
  float cmx = 0, cmy = 0, cmz = 0; // corrected and filtered external magnetometer or internal imu mag measurements in [uT]

  float timeElapsed = 0;              	// time difference with ts from last update() call [s] 
  uint32_t lastTime = 0;              // IMU sample timestamp [us]

protected:
  float B_gyr = 1.0;            // gyr filter constant
  float B_acc = 1.0;            // acc filter constant
  float B_mag = 1.0;            // mag filter constant
  
  bool hasMag = false;
  
  Filter *filter = nullptr;
  Quaternion* quat = nullptr;
  Imu9dMessage* imuData = nullptr;
};

 
