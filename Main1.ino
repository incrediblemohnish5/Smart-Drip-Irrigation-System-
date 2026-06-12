/*
  Smart Drip Irrigation for ESP32
  - Soil moisture: analog on D34
  - DHT22: data on D4
  - Water flow sensor: pulse on D5 (interrupt)
  - Relay controls: D15 (IN1), D2 (IN2) -> control 12V pump via relay COM/NO

  Logic:
  - Maintain moving average for soil moisture
  - If moisture below DRY threshold, start watering
  - Stop watering when moisture above WET threshold OR flow drops below minFlow
    OR maxWaterDuration reached
  - Print values and state changes to Serial

  Configurable constants below.
*/

#include <Arduino.h>
#include <DHT.h>

// Pins (use labels from the circuit)
const int PIN_SOIL = 34;     // analog input (ADC1_CH6)
const int PIN_DHT = 4;       // DHT22 data pin
const int PIN_FLOW = 5;      // pulse input from flow sensor
const int PIN_RELAY1 = 15;   // IN1 -> relay channel 1
const int PIN_RELAY2 = 2;    // IN2 -> relay channel 2

// DHT settings
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

// Relay polarity (set to true if your relay is active LOW)
const bool RELAY_ACTIVE_LOW = false;

// Soil moisture ADC settings
// ESP32 ADC range: 0-4095 (12-bit) by default; reading map may vary with attenuation
const int ADC_MAX = 4095;
const int SOIL_MOVING_AVG_SIZE = 10; // number of samples to average for soil
const unsigned long SOIL_SAMPLE_INTERVAL_MS = 2000; // sample soil every 2s

// Thresholds (these are raw ADC values; adjust for your sensor and installation)
// Typical: wetter soil -> higher or lower value depending on sensor. Verify and adjust.
// These defaults assume wetter => higher ADC (if not, swap logic or invert mapping)
const int SOIL_DRY_THRESHOLD = 1600;  // below this => dry
const int SOIL_WET_THRESHOLD = 2000;  // above this => wet enough to stop

// Flow sensor settings
volatile unsigned long flowPulseCount = 0; // incremented in ISR
unsigned long lastFlowCalcTime = 0;
float currentFlowLPerMin = 0.0; // computed flow (L/min)
const unsigned long FLOW_CALC_INTERVAL_MS = 5000; // compute flow every 5s
// pulses per liter for the flow sensor (typical YF-S201 ~450 pulses per liter)
const float FLOW_PULSES_PER_LITER = 450.0;
const float MIN_FLOW_LPM = 0.1; // if flow < this while watering, consider fault

// Watering control
bool watering = false;
unsigned long wateringStartTime = 0;
const unsigned long MAX_WATERING_DURATION_MS = 5UL * 60UL * 1000UL; // 5 minutes default

// Timing
unsigned long lastSoilSampleTime = 0;
unsigned long lastStatusPrintTime = 0;
const unsigned long STATUS_PRINT_INTERVAL_MS = 2000;

// Soil moving average buffer
int soilBuffer[SOIL_MOVING_AVG_SIZE];
int soilIndex = 0;
int soilCount = 0; // how many filled

// Forward
void startWatering(const char *reason);
void stopWatering(const char *reason);
void IRAM_ATTR flowPulseISR();
int readSoilRaw();
int getSoilAverage();

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Smart Drip Irrigation starting...");

  // Pins
  pinMode(PIN_RELAY1, OUTPUT);
  pinMode(PIN_RELAY2, OUTPUT);

  // Initialize relays to OFF
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(PIN_RELAY1, HIGH);
    digitalWrite(PIN_RELAY2, HIGH);
  } else {
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
  }

  // DHT
  dht.begin();

  // Flow sensor input
  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), flowPulseISR, FALLING);
  lastFlowCalcTime = millis();

  // Seed soil buffer with initial readings
  for (int i = 0; i < SOIL_MOVING_AVG_SIZE; ++i) {
    soilBuffer[i] = readSoilRaw();
  }
  soilCount = SOIL_MOVING_AVG_SIZE;
  soilIndex = 0;

  Serial.println("Setup complete.");
}

void loop() {
  unsigned long now = millis();

  // Soil sampling
  if (now - lastSoilSampleTime >= SOIL_SAMPLE_INTERVAL_MS) {
    lastSoilSampleTime = now;
    int raw = readSoilRaw();
    soilBuffer[soilIndex] = raw;
    soilIndex = (soilIndex + 1) % SOIL_MOVING_AVG_SIZE;
    if (soilCount < SOIL_MOVING_AVG_SIZE) soilCount++;
  }

  // Flow calculation periodically
  if (now - lastFlowCalcTime >= FLOW_CALC_INTERVAL_MS) {
    noInterrupts();
    unsigned long pulses = flowPulseCount;
    flowPulseCount = 0;
    interrupts();

    unsigned long dt = now - lastFlowCalcTime;
    lastFlowCalcTime = now;
    // L per minute = (pulses / pulses_per_liter) * (60000 / dt)
    if (dt > 0) {
      float liters = pulses / FLOW_PULSES_PER_LITER;
      currentFlowLPerMin = liters * (60000.0f / dt);
    } else {
      currentFlowLPerMin = 0.0;
    }
  }

  // Control logic executed after updating sensors
  int soilAvg = getSoilAverage();

  // Debug prints periodic
  if (now - lastStatusPrintTime >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusPrintTime = now;
    float humidity = NAN, temperature = NAN;
    // DHT is slow; read but tolerate failures
    humidity = dht.readHumidity();
    temperature = dht.readTemperature();

    Serial.print("Soil(avg): "); Serial.print(soilAvg);
    Serial.print("  Flow(L/min): "); Serial.print(currentFlowLPerMin, 2);
    Serial.print("  Watering: "); Serial.print(watering ? "ON" : "OFF");
    Serial.print("  DHT T:"); Serial.print(temperature); Serial.print("C H:"); Serial.println(humidity);
  }

  // Decision: start
  if (!watering && soilAvg < SOIL_DRY_THRESHOLD) {
    startWatering("Soil below dry threshold");
  }

  // Decision: stop conditions
  if (watering) {
    // 1) soil wet enough
    if (soilAvg >= SOIL_WET_THRESHOLD) {
      stopWatering("Soil reached wet threshold");
    }
    // 2) flow too low (possible blockage or pump failure)
    else if (currentFlowLPerMin < MIN_FLOW_LPM) {
      stopWatering("Flow below minimum threshold");
    }
    // 3) timeout
    else if (millis() - wateringStartTime >= MAX_WATERING_DURATION_MS) {
      stopWatering("Max watering duration reached");
    }
  }

  // Small delay to yield
  delay(10);
}

// --- Helper implementations ---

void startWatering(const char *reason) {
  watering = true;
  wateringStartTime = millis();
  // Activate relays
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
  } else {
    digitalWrite(PIN_RELAY1, HIGH);
    digitalWrite(PIN_RELAY2, HIGH);
  }
  Serial.print("START watering: "); Serial.println(reason);
}

void stopWatering(const char *reason) {
  watering = false;
  // Deactivate relays
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(PIN_RELAY1, HIGH);
    digitalWrite(PIN_RELAY2, HIGH);
  } else {
    digitalWrite(PIN_RELAY1, LOW);
    digitalWrite(PIN_RELAY2, LOW);
  }
  Serial.print("STOP watering: "); Serial.println(reason);
}

void IRAM_ATTR flowPulseISR() {
  // Basic debouncing not possible in ISR; assume sensor hardware clean pulses
  flowPulseCount++;
}

int readSoilRaw() {
  // Read ADC; perform a few quick samples and median/average to reduce noise
  const int SAMPLES = 5;
  long sum = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    int v = analogRead(PIN_SOIL);
    sum += v;
    delay(2);
  }
  int avg = sum / SAMPLES;
  return avg;
}

int getSoilAverage() {
  long s = 0;
  for (int i = 0; i < soilCount; ++i) s += soilBuffer[i];
  if (soilCount == 0) return readSoilRaw();
  return (int)(s / soilCount);
}
