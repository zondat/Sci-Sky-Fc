#pragma once

class Quaternion {
public:
  float q0=1.0f, q1=0.f, q2=0.f, q3=0.f;
  
  Quaternion() = default;
  Quaternion(float nq0, float nq1, float nq2, float nq3) {
	this->q0 = nq0;
	this->q1 = nq1;
	this->q2 = nq2;
	this->q3 = nq3;
  }
  virtual ~Quaternion() = default;
  
};