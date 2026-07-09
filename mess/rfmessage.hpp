#pragma once

#include "../config.h"

class RfMessage {
public:
  uint16_t channels[NB_CHANNELS];
  
  RfMessage() {
	for (int i=0; i<NB_CHANNELS; i++) {
		channels[i] = 1500;
	}
	channels[CHN_THRUST] = PPM_MIN;
	channels[CHN_ARM] = PPM_MIN;		
  }
};