#include "pid.hpp"

/**
 * @brief Lớp PIFF (PID với Feed Forward) kế thừa từ PID
 * Có hai phiên bản compute tương tự như PID
 */
class PIFF : public PID {
private:
    float kff; // Hệ số Feed Forward

public:
    PIFF(float kp, float ki, float kd, float kff,
         float outputMin, float outputMax,
         float integralMin = 0, float integralMax = 0) 
        : PID(kp, ki, kd, outputMin, outputMax, integralMin, integralMax), kff(kff) {}
    
    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        
        integral += error * dt;
        integral = constrain(integral, integralMin, integralMax);
        
        float derivative = (error - lastError) / dt;
        float derivativeFiltered = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * derivative;
        lastDerivative = derivativeFiltered;
        
        float output = kp * error + ki * integral + kd * derivativeFiltered + kff * setpoint;
        output = constrain(output, outputMin, outputMax);
        
        lastError = error;
        return output;
    }
    
    float compute(float setpoint, float measurement, float dt, float externalDerivative) {
        float error = setpoint - measurement;
        
        integral += error * dt;
        integral = constrain(integral, integralMin, integralMax);
        
        float derivativeFiltered = DERIVATIVE_FILTER * lastDerivative + (1.0 - DERIVATIVE_FILTER) * externalDerivative;
        lastDerivative = derivativeFiltered;
        
        float output = kp * error + ki * integral + kd * derivativeFiltered + kff * setpoint;
        output = constrain(output, outputMin, outputMax);
        
        lastError = error;
        return output;
    }
        
    float getFF() const { return kff; }
};