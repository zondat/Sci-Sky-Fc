#pragma once

#ifndef CONFIG_H
  #define CONFIG_H

#include <Arduino.h>

// Radio link
#define IBUS_PROTOCOL_CHANNELS  14
#define NB_CHANNELS             16
#define PPM_MAX                 2000
#define PPM_MIN                 1000
#define CHN_AILERON             1
#define CHN_ELEVATOR            2
#define CHN_THRUST              3
#define CHN_RUDDER              4
#define CHN_ARM                 9

// MCU
#define NB_PWM_CHN              5

#define PIN_THR                 9
#define PIN_PITCH               8
#define PIN_AIL_1               0
#define PIN_AIL_2               3

#define PIN_MOSI                4
#define PIN_MISO                5
#define PIN_SCK                 2
#define PIN_SS                  10

#define PIN_TX                  -1
#define PIN_RX                  20

// Task config
#define DISPATCHER_STACK_SIZE   2048
#define DISPATCHER_PRIORITY     2      
#define CONTROL_TASK_PRIORITY   1
#define CONTROL_INTERVAL_US     3000  // us
#define IMU_UPDATE_INTERVAL_MS  3     // ms
#define RX_SCANNER_INTERVAL_MS  3     // ms 
#define TRIGGER_JITTER_US       200   // ms

// IMU config
#define IMU_RATE			          1000
#define IMU_ACC_LP			  	    70  	// lowpass filter cutoff frequency for accelerator 
#define IMU_GYR_LP			  	    60  	// lowpass filter cutoff frequency for gyro
#define IMU_MAG_LP			  	    1e10  // lowpass filter cutoff frequency for magnetometer

#define IMU_ACC_OFFSET_X		    0
#define IMU_ACC_OFFSET_Y		    0
#define IMU_ACC_OFFSET_Z		    0

#define IMU_GYR_OFFSET_X		    0
#define IMU_GYR_OFFSET_Y		    0
#define IMU_GYR_OFFSET_Z		    0

#define IMU_MAG_OFFSET_X		    0
#define IMU_MAG_OFFSET_Y		    0
#define IMU_MAG_OFFSET_Z		    0	

#define IMU_MAG_SCALE_X		      1
#define IMU_MAG_SCALE_Y		      1
#define IMU_MAG_SCALE_Z		      1	

#define I_LIMIT        			    25          //PID Integrator saturation level, mostly for safety
#define MAX_ROLL        		    45.0        //Max roll angle in degrees for ROLL, FBWA modes
#define MAX_PITCH       		    30.0        //Max pitch angle in degrees for FBWA mode

//roll PID constants
#define Kp_roll           		  15          //Roll P-gain - apply full aileron on 90 degree roll error
#define Ki_roll           		  0.5           //Roll I-gain
#define Kd_roll           		  1.75           //Roll D-gain - apply full opposite aileron when roll rate is 180 degrees/sec towards desired setpoint
#define Kf_roll                 25

//pitch PID constants
#define Kp_pitch          		  15          //Pitch P-gain - apply full elevator on 30 degree pitch error
#define Ki_pitch          		  0.5           //Pitch I-gain
#define Kd_pitch          		  1.5           //Pitch D-gain - apply full opposite elevator when pitch rate is 90 degrees/sec towards desired setpoint
#define Kf_pitch                25

#define rad_to_deg              57.2957795132f
#define deg_to_rad              0.0174532925199f

#define DEBUG                   1

#endif