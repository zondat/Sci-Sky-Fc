#pragma once

#if defined(NRF52840_XXAA)
  #include <FreeRTOS.h>
  #include <task.h>
  #include <semphr.h>
#endif

class TimeTable {
protected:
  SemaphoreHandle_t semWakeup = xSemaphoreCreateBinary();   
  SemaphoreHandle_t semElapsedTime = xSemaphoreCreateBinary();   
  uint32_t lastWakeupUs = 0;
  uint32_t elapsedTimeUs = 0;

public:
  TimeTable(uint32_t lwu = 0, uint32_t dt = 0) {
    lastWakeupUs = lwu;
    elapsedTimeUs = dt;
    semWakeup = xSemaphoreCreateBinary();   
    semElapsedTime = xSemaphoreCreateBinary();  
    xSemaphoreGive(semWakeup);
    xSemaphoreGive(semElapsedTime);
  }

  uint32_t getLastWakeupUs() volatile {
    uint32_t val = -1;
    if (xSemaphoreTake(semWakeup, portMAX_DELAY) == pdTRUE) {
      val = lastWakeupUs;
      xSemaphoreGive(semWakeup);
    }
    return val;
  }

  void setLastWakeupUs(uint32_t value) volatile {
    if (xSemaphoreTake(semWakeup, portMAX_DELAY) == pdTRUE) {
      lastWakeupUs = value;
      xSemaphoreGive(semWakeup);
    }    
  }

  uint32_t getElapsedTimeUs() volatile {
    uint32_t val = -1;
    if (xSemaphoreTake(semElapsedTime, portMAX_DELAY) == pdTRUE) {
      val = elapsedTimeUs;
      xSemaphoreGive(semElapsedTime);
    }
    return val;
  }

  void setElapsedTimeUs(uint32_t value) volatile {
    if (xSemaphoreTake(semElapsedTime, portMAX_DELAY) == pdTRUE) {
      elapsedTimeUs = value;
      xSemaphoreGive(semElapsedTime);  
    }      
  }
};