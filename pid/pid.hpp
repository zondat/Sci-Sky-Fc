#include <Arduino.h>

class PID {
protected:
    float kp, ki, kd;
    float outputMin, outputMax;
    float integralMin, integralMax;
    float integral = 0.0;
    float lastError = 0.0;
    float lastDerivative = 0.0;
    const float DERIVATIVE_FILTER = 0.9;

public:
    PID(float kp, float ki, float kd, 
        float outputMin, float outputMax,
        float integralMin = 0, float integralMax = 0) 
        : kp(kp), ki(ki), kd(kd), 
          outputMin(outputMin), outputMax(outputMax) {
        if (integralMin == 0 && integralMax == 0) {
            this->integralMin = outputMin * 0.8;
            this->integralMax = outputMax * 0.8;
        } else {
            this->integralMin = integralMin;
            this->integralMax = integralMax;
        }
    }
    
    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        integral += error * dt;
        integral = constrain(integral, integralMin, integralMax);
        
        float derivative = (error - lastError) / dt;
        float derivativeFiltered = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * derivative;
        lastDerivative = derivativeFiltered;
        
        float output = kp * error + ki * integral + kd * derivativeFiltered;
        output = constrain(output, outputMin, outputMax);
        
        lastError = error;
        return output;
    }
    
    float compute(float setpoint, float measurement, float dt, float externalDerivative) {
        float error = setpoint - measurement;
        
        integral += error * dt;
        integral = constrain(integral, integralMin, integralMax);
        
        // Sử dụng external derivative (đã được lọc ở bên ngoài)
        float derivativeFiltered = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * externalDerivative;
        lastDerivative = derivativeFiltered;
        
        float output = kp * error + ki * integral + kd * derivativeFiltered;
        output = constrain(output, outputMin, outputMax);
        
        lastError = error;
        return output;
    }
    
    void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastDerivative = 0.0;
    }
};