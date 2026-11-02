// EV Battery Thermal Management Controller — ESP32 + FreeRTOS
// Wokwi simulation project. No delay() in tasks; every task is periodic or
// event-driven so the scheduler stays responsive.
//
// Hardware (see diagram.json):
//   NTC thermistor module  -> GPIO 34 (ADC)
//   Fan / radiator PWM     -> GPIO 25 (red LED)
//   Coolant pump PWM       -> GPIO 26 (green LED)
//   Over-temperature alarm -> GPIO 27 (buzzer)
//   Fault acknowledge      -> GPIO 4  (button to GND, INPUT_PULLUP)

#include <Arduino.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// ---------------- configuration ----------------
static constexpr uint8_t PIN_NTC   = 34;
static constexpr uint8_t PIN_FAN   = 25;
static constexpr uint8_t PIN_PUMP  = 26;
static constexpr uint8_t PIN_ALARM = 27;
static constexpr uint8_t PIN_ACK   = 4;

static constexpr uint32_t SAMPLE_PERIOD_MS   = 100;   // sensor task
static constexpr uint32_t CONTROL_PERIOD_MS  = 50;    // control task
static constexpr uint32_t TELEMETRY_PERIOD_MS = 500;  // serial task

// Thermal thresholds (degrees Celsius).
static constexpr float FAN_ON_C   = 42.0f;
static constexpr float FAN_OFF_C  = 38.0f;
static constexpr float PUMP_ON_C  = 48.0f;
static constexpr float PUMP_OFF_C = 44.0f;
static constexpr float OVERTEMP_C = 75.0f;

// NTC module: 10K thermistor in series with 10K, beta 3950.
static constexpr float NTC_BETA = 3950.0f;
static constexpr float ADC_MAX  = 4095.0f;  // ESP32 12-bit ADC

// ---------------- shared state ----------------
struct SystemState {
  float tempC;
  uint8_t fanDuty;    // 0..255
  uint8_t pumpDuty;   // 0..255
  bool overTemp;      // latched until acknowledged
};

static SystemState g_state = {25.0f, 0, 0, false};
static SemaphoreHandle_t g_stateMutex;
static QueueHandle_t g_sampleQueue;  // float, overwrite semantics

// ---------------- helpers ----------------
static float ntcToCelsius(int raw) {
  // Guard the log() domain at both rails.
  if (raw <= 0) raw = 1;
  if (raw >= (int)ADC_MAX) raw = (int)ADC_MAX - 1;

  const float ratio = ADC_MAX / (float)raw - 1.0f;
  const float invT  = logf(ratio) / NTC_BETA + (1.0f / 298.15f);
  return (1.0f / invT) - 273.15f;
}

static float readTemperature() {
  // Median of five rejects single-sample ADC spikes.
  int s[5];
  for (int i = 0; i < 5; i++) {
    s[i] = analogRead(PIN_NTC);
    delayMicroseconds(200);  // tiny, bounded; not a blocking task delay
  }
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (s[j] < s[i]) {
        int t = s[i];
        s[i] = s[j];
        s[j] = t;
      }
    }
  }
  return ntcToCelsius(s[2]);
}

static void applyOutputs(uint8_t fan, uint8_t pump, bool alarm) {
  analogWrite(PIN_FAN, fan);
  analogWrite(PIN_PUMP, pump);
  digitalWrite(PIN_ALARM, alarm ? HIGH : LOW);
}

// ---------------- tasks ----------------
// 1. Acquisition: sample the NTC and publish the newest value.
static void sensorTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    const float t = readTemperature();
    xQueueOverwrite(g_sampleQueue, &t);  // keep only the latest sample
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
  }
}

// 2. Control: hysteresis + proportional fan, over-temperature latch.
static void controlTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t overTempSinceMs = 0;

  for (;;) {
    float t = NAN;
    if (xQueueReceive(g_sampleQueue, &t, pdMS_TO_TICKS(CONTROL_PERIOD_MS)) == pdTRUE) {
      xSemaphoreTake(g_stateMutex, portMAX_DELAY);

      g_state.tempC = t;

      if (!g_state.overTemp && t >= OVERTEMP_C) {
        if (overTempSinceMs == 0) overTempSinceMs = millis();
        if (millis() - overTempSinceMs >= 1000) {  // 1 s persistence
          g_state.overTemp = true;
          Serial.println("EV:FAULT over-temperature latched");
        }
      } else if (t < OVERTEMP_C) {
        overTempSinceMs = 0;
      }

      if (g_state.overTemp) {
        // Fail safe: full cooling, alarm on.
        g_state.fanDuty  = 255;
        g_state.pumpDuty = 255;
      } else {
        // Fan: proportional between FAN_OFF_C and 70 C.
        if (t >= FAN_ON_C || (g_state.fanDuty > 0 && t >= FAN_OFF_C)) {
          const float span = 70.0f - FAN_OFF_C;
          float pct = (t - FAN_OFF_C) / span;
          if (pct < 0.25f) pct = 0.25f;  // never run a stalled fan
          if (pct > 1.0f) pct = 1.0f;
          g_state.fanDuty = (uint8_t)(pct * 255.0f);
        } else {
          g_state.fanDuty = 0;
        }

        // Pump: simple hysteresis.
        if (t >= PUMP_ON_C) {
          g_state.pumpDuty = 255;
        } else if (t < PUMP_OFF_C) {
          g_state.pumpDuty = 0;
        }
      }

      const uint8_t fan  = g_state.fanDuty;
      const uint8_t pump = g_state.pumpDuty;
      const bool alarm   = g_state.overTemp;
      xSemaphoreGive(g_stateMutex);

      applyOutputs(fan, pump, alarm);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

// 3. Telemetry: one CSV line per period, easy to plot or paste into a report.
static void telemetryTask(void *) {
  TickType_t lastWake = xTaskGetTickCount();
  Serial.println("ms,temp_c,fan_pct,pump_pct,fault");

  for (;;) {
    xSemaphoreTake(g_stateMutex, portMAX_DELAY);
    const float t     = g_state.tempC;
    const uint8_t fan = g_state.fanDuty;
    const uint8_t pump = g_state.pumpDuty;
    const bool fault  = g_state.overTemp;
    xSemaphoreGive(g_stateMutex);

    Serial.printf("%lu,%.1f,%u,%u,%d\n",
                  (unsigned long)millis(), t,
                  (unsigned)(fan * 100 / 255),
                  (unsigned)(pump * 100 / 255),
                  fault ? 1 : 0);

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
  }
}

// 4. Input: debounced acknowledge; clears a latched fault.
static void inputTask(void *) {
  bool lastRaw = HIGH;
  bool stable  = HIGH;
  uint32_t lastChange = 0;
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    const bool raw = digitalRead(PIN_ACK);
    if (raw != lastRaw) {
      lastRaw = raw;
      lastChange = millis();
    }
    if (millis() - lastChange >= 30 && raw != stable) {
      stable = raw;
      if (stable == LOW) {
        xSemaphoreTake(g_stateMutex, portMAX_DELAY);
        if (g_state.overTemp) {
          g_state.overTemp = false;
          Serial.println("EV:ACK fault cleared");
        }
        xSemaphoreGive(g_stateMutex);
      }
    }
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
  }
}

// ---------------- setup ----------------
void setup() {
  Serial.begin(115200);
  delay(200);  // allow USB serial to attach before the first line

  pinMode(PIN_ALARM, OUTPUT);
  pinMode(PIN_ACK, INPUT_PULLUP);
  digitalWrite(PIN_ALARM, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_NTC, ADC_11db);
  // ESP32 Arduino core 3.x takes the pin as the first argument.
  analogWriteResolution(PIN_FAN, 8);
  analogWriteResolution(PIN_PUMP, 8);
  analogWrite(PIN_FAN, 0);
  analogWrite(PIN_PUMP, 0);

  g_stateMutex  = xSemaphoreCreateMutex();
  g_sampleQueue = xQueueCreate(1, sizeof(float));
  configASSERT(g_stateMutex && g_sampleQueue);

  xTaskCreatePinnedToCore(sensorTask,    "sensor",    4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(controlTask,   "control",   4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(inputTask,     "input",     2048, nullptr, 1, nullptr, 0);

  Serial.println("EV thermal controller ready");
}

void loop() {
  // All work happens in tasks; keep the Arduino loop idle.
  vTaskDelay(pdMS_TO_TICKS(1000));
}
