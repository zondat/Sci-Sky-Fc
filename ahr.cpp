#include "ahr.hpp"
#include "ahr/mahony.hpp"

#include <math.h>
// float constrain(float amt, float low, float high) {
//   return (amt < low) ? low : (amt > high ? high:amt );
// } 
float lowpass_to_beta(float f0, float fs) {
  return constrain(1 - exp(-2 * PI * f0 / fs), 0.0f, 1.0f);
}

bool Ahr::setup() {
  B_gyr = lowpass_to_beta(IMU_GYR_LP, IMU_RATE);
  B_acc = lowpass_to_beta(IMU_ACC_LP, IMU_RATE);
  B_mag = lowpass_to_beta(IMU_MAG_LP, IMU_RATE);
  
  quat = new Quaternion(1, 0, 0, 0);
  filter = new Mahony(quat);
  return true;
}

void Ahr::update() {
  //Low-pass filtered, corrected accelerometer data
  cax += B_acc * ((imuData->ax - IMU_ACC_OFFSET_X) - cax);
  cay += B_acc * ((imuData->ay - IMU_ACC_OFFSET_Y) - cay);
  caz += B_acc * ((imuData->az - IMU_ACC_OFFSET_Z) - caz);

  //Low-pass filtered, corrected gyro data
  cgx += B_gyr * ((imuData->gx - IMU_GYR_OFFSET_X) - cgx);
  cgy += B_gyr * ((imuData->gy - IMU_GYR_OFFSET_Y) - cgy);
  cgz += B_gyr * ((imuData->gz - IMU_GYR_OFFSET_Z) - cgz);

  //Magnetometer (External chip, or internal in IMU chip) 
  if(hasMag) {
    float _mx = imuData->mx;
    float _my = imuData->my;
    float _mz = imuData->mz;
    //update the mag values
    if( ! (_mx == 0 && _my == 0 && _mz == 0) ) {
      //Correct the mag values with the calibration values
      _mx = (_mx - IMU_MAG_OFFSET_X) * IMU_MAG_SCALE_X;
      _my = (_my - IMU_MAG_OFFSET_Y) * IMU_MAG_SCALE_Y;
      _mz = (_mz - IMU_MAG_OFFSET_Z) * IMU_MAG_SCALE_Z;
      //Low-pass filtered magnetometer data
      cmx += B_mag * (_mx - cmx);
      cmy += B_mag * (_my - cmy);
      cmz += B_mag * (_mz - cmz);
    }
  } else {
    cmx = 0;
    cmy = 0;
    cmz = 0;
  }

  //update dt and ts for use by gizmo
  uint32_t currentTime = micros();
  timeElapsed = ((float)(currentTime - lastTime)) * 1e-6;
  lastTime = currentTime;

  //call gizmo to update q
  filter->update(cgx * deg_to_rad, 
                  cgy * deg_to_rad, 
                  cgz * deg_to_rad, 
                  cax, cay, caz, 
                  cmx, cmy, cmz, timeElapsed);

  //update euler angles
  updateEulerAngles();
}

void Ahr::init() {
  //update mx and my from mag or imu
  update();

  //calculate yaw angle
  if(cmx <= 0 && cmy <= 0 && cmz <= 0) {
    yaw = 0;
    pitch = 0;
    roll = 0;
    quat = new Quaternion(1, 0, 0, 0);
  } else {
    float yaw_rad = -atan2(cmy, cmx);
    yaw = yaw_rad * rad_to_deg;
    pitch = 0;
    roll = 0;
    quat = new Quaternion(cos(yaw_rad/2), 0, 0, sin(yaw_rad/2));
  }
}

void Ahr::updateEulerAngles() {
  roll 	= atan2(quat->q0 * quat->q1 + quat->q2 * quat->q3, 0.5f - quat->q1 * quat->q1 - quat->q2 * quat->q2) * rad_to_deg; //degrees - roll right is positive
  pitch = asin(constrain(-2.0f * (quat->q1 * quat->q3 - quat->q0 * quat->q2), -1.0, 1.0)) * rad_to_deg; //degrees - pitch up is positive - use constrain() to prevent NaN due to rounding
  yaw 	= atan2(quat->q1 * quat->q2 + quat->q0 * quat->q3, 0.5f - quat->q2 * quat->q2 - quat->q3 * quat->q3) * rad_to_deg; //degrees - yaw right is positive
}

//get acceleration in earth-frame up direction in [m/s^2]
float Ahr::getAccelUp() {
  return 9.80665 * (
					(2 * quat->q1 * quat->q3 - 2 * quat->q0 * quat->q2) * cax + 
					(2 * quat->q2 * quat->q3 + 2 * quat->q0 * quat->q1) * cay + 
					(quat->q0 * quat->q0 - quat->q1 * quat->q1 - quat->q2 * quat->q2 + quat->q3 * quat->q3) * caz - 1.0 );
}
