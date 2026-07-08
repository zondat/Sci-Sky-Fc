#pragma once
#include "filter.hpp"
#include <math.h> //sqrtf

class Mahony : public Filter {
public:
  Mahony(Quaternion *q) : Filter(q) {}
  virtual ~Mahony(){ delete q; }
  
  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) override {
    if( (mx <= 0.0f) && (my <= 0.0f) && (mz <= 0.0f) ) {
	  update6DOF(gx, gy, gz, ax, ay, az, dt);
    } else {
	  update9DOF(gx, gy, gz, ax, ay, az, mx, my, mz, dt);
    }
  }

protected:
  float config_2Kp = 2 * 0.5;       //2 * proportional gain (Kp)
  float config_2Ki = 2 * 0.0;       //2 * integral gain (Ki)
  float config_alen2_min = 0;       //default 0, betaflight uses 0.81
  float config_alen2_max = 1e10;    //default 1e10, betaflight uses 1.21

  float integralFBx = 0, integralFBy = 0, integralFBz = 0;  // integral error terms scaled by Ki
  
  void update9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    //Compute feedback only if accelerometer measurement is in range 0.9g - 1.1g
    float alen2 = ax * ax + ay * ay + az * az;

    if (alen2 > config_alen2_min && alen2 < config_alen2_max) {

	  //Normalise accelerometer measurement
	  recipNorm = 1.0 / sqrtf(alen2);
	  ax *= recipNorm;
	  ay *= recipNorm;
	  az *= recipNorm;

	  // Normalise magnetometer measurement
	  recipNorm = 1.0f/sqrtf(mx * mx + my * my + mz * mz);
	  mx *= recipNorm;
	  my *= recipNorm;
	  mz *= recipNorm;

	  // Auxiliary variables to avoid repeated arithmetic
	  q0q0 = q->q0 * q->q0;
	  q0q1 = q->q0 * q->q1;
	  q0q2 = q->q0 * q->q2;
	  q0q3 = q->q0 * q->q3;
	  q1q1 = q->q1 * q->q1;
	  q1q2 = q->q1 * q->q2;
	  q1q3 = q->q1 * q->q3;
	  q2q2 = q->q2 * q->q2;
	  q2q3 = q->q2 * q->q3;
	  q3q3 = q->q3 * q->q3;

	  // Reference direction of Earth's magnetic field
	  hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
	  hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
	  bx = sqrtf(hx * hx + hy * hy);
	  bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

	  // Estimated direction of gravity and magnetic field
	  halfvx = q1q3 - q0q2;
	  halfvy = q0q1 + q2q3;
	  halfvz = q0q0 - 0.5f + q3q3;
	  halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
	  halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
	  halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

	  // Error is sum of cross product between estimated direction
	  // and measured direction of field vectors
	  halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
	  halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
	  halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

	  // Compute and apply integral feedback if enabled
	  if(config_2Ki > 0.0f) {
	    // integral error scaled by Ki
	    integralFBx += config_2Ki * halfex * dt;
	    integralFBy += config_2Ki * halfey * dt;
	    integralFBz += config_2Ki * halfez * dt;
	    gx += integralFBx;	// apply integral feedback
	    gy += integralFBy;
	    gz += integralFBz;
	  } else {
	    integralFBx = 0.0f;	// prevent integral windup
	    integralFBy = 0.0f;
	    integralFBz = 0.0f;
	  }

	  // Apply proportional feedback
	  gx += config_2Kp * halfex;
	  gy += config_2Kp * halfey;
	  gz += config_2Kp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= 0.5f * dt; // pre-multiply common factors
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q->q0;
    qb = q->q1;
    qc = q->q2;
    q->q0 += (-qb * gx - qc * gy - q->q3 * gz);
    q->q1 += (qa * gx + qc * gz - q->q3 * gy);
    q->q2 += (qa * gy - qb * gz + q->q3 * gx);
    q->q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = 1.0f/sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    q->q0 *= recipNorm;
    q->q1 *= recipNorm;
    q->q2 *= recipNorm;
    q->q3 *= recipNorm;
  }
  
  void update6DOF(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	//Compute feedback only if accelerometer measurement is in range 0.9g - 1.1g
	float alen2 = ax * ax + ay * ay + az * az;

	if (alen2 > config_alen2_min && alen2 < config_alen2_max) {
	  //Normalise accelerometer measurement
	  recipNorm = 1.0 / sqrtf(alen2);
	  ax *= recipNorm;
	  ay *= recipNorm;
	  az *= recipNorm;

	  // Estimated direction of gravity
	  halfvx = q->q1 * q->q3 - q->q0 * q->q2;
	  halfvy = q->q0 * q->q1 + q->q2 * q->q3;
	  halfvz = q->q0 * q->q0 - 0.5f + q->q3 * q->q3;

	  // Error is sum of cross product between estimated
	  // and measured direction of gravity
	  halfex = (ay * halfvz - az * halfvy);
	  halfey = (az * halfvx - ax * halfvz);
	  halfez = (ax * halfvy - ay * halfvx);

	  // Compute and apply integral feedback if enabled
	  if(config_2Ki > 0.0f) {
	    // integral error scaled by Ki
	    integralFBx += config_2Ki * halfex * dt;
	    integralFBy += config_2Ki * halfey * dt;
	    integralFBz += config_2Ki * halfez * dt;
	    gx += integralFBx;	// apply integral feedback
	    gy += integralFBy;
	    gz += integralFBz;
	  } else {
	    integralFBx = 0.0f;	// prevent integral windup
	    integralFBy = 0.0f;
	    integralFBz = 0.0f;
	  }

	  // Apply proportional feedback
	  gx += config_2Kp * halfex;
	  gy += config_2Kp * halfey;
	  gz += config_2Kp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= 0.5f * dt; // pre-multiply common factors
	gy *= 0.5f * dt;
	gz *= 0.5f * dt;
	qa = q->q0;
	qb = q->q1;
	qc = q->q2;
	q->q0 += (-qb * gx - qc * gy - q->q3 * gz);
	q->q1 += (qa * gx + qc * gz - q->q3 * gy);
	q->q2 += (qa * gy - qb * gz + q->q3 * gx);
	q->q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = 1.0f/sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
	q->q0 *= recipNorm;
	q->q1 *= recipNorm;
	q->q2 *= recipNorm;
	q->q3 *= recipNorm;
  }
};
