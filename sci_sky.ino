#include "fc.hpp"

Fc *fc = nullptr;

// ========== HANDLE & SEMAPHORE ==========
TaskHandle_t imuTaskHandle = NULL;
TaskHandle_t rxTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;

// ========== TASKS ==========
void imuUpdate(void *pvParameters);
void rxUpdate(void *pvParameters);
void controlFlight(void *pvParameters);

// ========== Tasks implementation ==========
void imuUpdate(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(IMU_UPDATE_INTERVAL_MS);
  
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    fc->imuUpdate();
    // Serial.println("imuUpdate()");
  }
}

void rxUpdate(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(RX_SCANNER_INTERVAL_MS);
  Serial.println("rxUpdate()");
  
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    fc->rxUpdate();
  }
}

void controlFlight(void *pvParameters) {
  while (1) {
    fc->controlFlight();
  }
}

void setup() {
  if (DEBUG) {
    while (!Serial) {
      Serial.begin(115200);
      delay(200);
    }
    Serial.println("Serial ok");
  }

  fc = new Fc();
  Serial.println("Init Fc");
  if (!fc->setup()) return;
  fc->calib();
  fc->update();

  if (DEBUG) Serial.println("Start");

  xTaskCreate(
    imuUpdate,
    "IMU Update",
    2048,
    NULL,
    1,
    &imuTaskHandle
  );

  // Sensor Task: Core 1, Priority 3 (ưu tiên cao hơn Dispatcher)
  xTaskCreate(
    rxUpdate,
    "Rx",
    2048,
    NULL,
    3,
    &rxTaskHandle
  );

  // Control Task: Core 1, Priority 4 (ưu tiên cao nhất)
  xTaskCreate(
    controlFlight,
    "Control",
    4096,
    NULL,
    4,
    &controlTaskHandle
  );
}

void loop() {
  delay(1000);
}
