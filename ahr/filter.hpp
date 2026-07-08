#pragma once

#include "../mess/quaternion.hpp"
#include "../mess/imu9dmessage.hpp"

class Filter {
public:
	Filter(Quaternion* q) {this->q = q;}
	virtual void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) = 0;
	
	Quaternion* q = nullptr;
};