class TimeTable {
public:
  TimeTable(uint32_t lwu = 0, uint32_t dt = 0) {
    lastWakeup = lwu;
    elapsedTimeMs = dt;
    semWakeup = xSemaphoreCreateBinary();   
    semElapsedTime = xSemaphoreCreateBinary();  
    xSemaphoreGive(semWakeup);
    xSemaphoreGive(semElapsedTime);
  }

  uint32_t getLastWakeup() volatile {
    uint32_t val = -1;
    if (xSemaphoreTake(semWakeup, portMAX_DELAY) == pdTRUE) {
      val = lastWakeup;
      xSemaphoreGive(semWakeup);
    }
    return val;
  }

  void setLastWakeup(uint32_t value) volatile {
    if (xSemaphoreTake(semWakeup, portMAX_DELAY) == pdTRUE) {
      lastWakeup = value;
      xSemaphoreGive(semWakeup);
    }    
  }

  uint32_t getElapsedTimeMs() volatile {
    uint32_t val = -1;
    if (xSemaphoreTake(semElapsedTime, portMAX_DELAY) == pdTRUE) {
      val = elapsedTimeMs;
      xSemaphoreGive(semElapsedTime);
    }
    return val;
  }

  void setElapsedTimeMs(uint32_t value) volatile {
    if (xSemaphoreTake(semElapsedTime, portMAX_DELAY) == pdTRUE) {
      elapsedTimeMs = value;
      xSemaphoreGive(semElapsedTime);  
    }      
  }

protected:
  #ifdef ARDUINO_ARCH_ESP32
    SemaphoreHandle_t semWakeup = xSemaphoreCreateBinary();   
    SemaphoreHandle_t semElapsedTime = xSemaphoreCreateBinary();   
  #endif
  uint32_t lastWakeup = 0;
  uint32_t elapsedTimeMs = 0;
};