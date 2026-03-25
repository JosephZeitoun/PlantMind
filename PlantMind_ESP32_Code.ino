// PlantMind - Sensor + Pump + Firebase Upload
// Heltec LoRa32 V3 (ESP32-S3)
// KY-028 Temp → GPIO3 (AO), Moisture → GPIO4, Relay → GPIO6
// Sends sensor readings to Firebase Realtime Database every 60 seconds

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <time.h>
#include <math.h>

// ─── WIFI CREDENTIALS ───────────────────────────────────────────
#define WIFI_SSID     "VM4738906"
#define WIFI_PASSWORD "xx2ttxJpfufkyjen"

// ─── FIREBASE CONFIG ────────────────────────────────────────────
#define FIREBASE_URL  "https://plantmind-18003-default-rtdb.firebaseio.com"

// ─── PIN DEFINITIONS ────────────────────────────────────────────
#define TEMP_PIN      3    // KY-028 analog output (AO)
#define MOISTURE_PIN  4    // Capacitive soil moisture sensor
#define RELAY_PIN     6    // Relay control

// ─── SETTINGS ───────────────────────────────────────────────────
#define ADC_MAX          4095
#define MOISTURE_LOW     53.5    // Pump triggers below this %
#define MOISTURE_SAMPLES 5       // Readings to average per cyclenn
#define RELAY_ON         HIGH
#define RELAY_OFF        LOW
#define UPLOAD_INTERVAL  60000   // Upload every 60 seconds (ms)
#define READ_INTERVAL    2000    // Read sensors every 2 seconds (ms)
#define PUMP_DURATION    2000    // Pump runs for 2 seconds max (ms)
#define PUMP_COOLDOWN    30000   // Wait 30 seconds before pumping again (ms)

// ─── KY-028 THERMISTOR CONSTANTS (Steinhart-Hart) ───────────────
#define THERMISTOR_NOMINAL   100000  // Resistance at 25°C (100kΩ — typical for KY-028)
#define TEMPERATURE_NOMINAL  25.0    // Reference temperature (°C)
#define B_COEFFICIENT        3950    // Beta coefficient of thermistor
#define SERIES_RESISTOR      100000  // Series resistor on the module (100kΩ)
#define TEMP_OFFSET          -33.0   // Calibration offset — adjust if reading is off

// ─── STATE ──────────────────────────────────────────────────────
bool pumpRunning      = false;
bool pumpStateChanged = false;
unsigned long lastUpload   = 0;
unsigned long lastRead     = 0;
unsigned long pumpStarted  = 0;   // When the pump was turned on
unsigned long lastPumpEnd  = 0;   // When the pump last finished (for cooldown)
float lastMoisture    = 0;
float lastTemperature = 0;

// ─── WIFI CONNECT ───────────────────────────────────────────────
void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());

    // Sync time via NTP (needed for timestamps)
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    Serial.print("Syncing time");
    time_t now = 0;
    int timeAttempts = 0;
    while (now < 1700000000 && timeAttempts < 20) {
      delay(500);
      Serial.print(".");
      time(&now);
      timeAttempts++;
    }
    Serial.println(" OK");
  } else {
    Serial.println();
    Serial.println("WiFi FAILED — running offline (pump still works)");
  }
}

// ─── READ KY-028 TEMPERATURE ────────────────────────────────────
// Converts raw ADC value from the thermistor into Celsius using
// the Steinhart-Hart simplified B-parameter equation.
float readTemperature() {
  // Average multiple samples to reduce noise
  long sum = 0;
  for (int i = 0; i < MOISTURE_SAMPLES; i++) {
    sum += analogRead(TEMP_PIN);
    delay(10);
  }
  float raw = sum / (float)MOISTURE_SAMPLES;

  // Debug: uncomment to see raw ADC value
  // Serial.print("[DEBUG] Temp ADC raw: ");
  // Serial.println(raw, 1);

  // Convert ADC to resistance
  // KY-028 module: thermistor to VCC, series resistor to GND
  if (raw <= 0 || raw >= ADC_MAX) return -999;  // Bad reading guard
  float resistance = SERIES_RESISTOR * (raw / (ADC_MAX - raw));

  // Serial.print("[DEBUG] Resistance: ");
  // Serial.println(resistance, 0);

  // Steinhart-Hart B-parameter equation
  float steinhart = resistance / THERMISTOR_NOMINAL;       // R/Ro
  steinhart = log(steinhart);                               // ln(R/Ro)
  steinhart /= B_COEFFICIENT;                              // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);       // + (1/To)
  steinhart = 1.0 / steinhart;                             // Invert
  float tempValue = steinhart - 273.15;

  // Apply calibration offset
  float celsius = tempValue + TEMP_OFFSET;

  return celsius;
}

// ─── SEND DATA TO FIREBASE ──────────────────────────────────────
void sendToFirebase(float moisture, float temperature, bool pumpOn) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected — skipping upload");
    connectWiFi();
    return;
  }

  time_t now;
  time(&now);

  // Build JSON — field names must match what the dashboard reads
  String json = "{";
  json += "\"moisture\":"    + String(moisture, 1)     + ",";
  json += "\"temperature\":" + String(temperature, 1)  + ",";
  json += "\"pumpOn\":"      + String(pumpOn ? "true" : "false") + ",";
  json += "\"timestamp\":"   + String((unsigned long)now);
  json += "}";

  HTTPClient http;
  String url = String(FIREBASE_URL) + "/readings.json";
  http.begin(url);
  http.setTimeout(5000);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST(json);

  if (httpCode == 200) {
    Serial.println("Firebase: uploaded OK");
  } else {
    Serial.print("Firebase: FAILED (HTTP ");
    Serial.print(httpCode);
    Serial.println(")");
    Serial.println(http.getString());
  }

  http.end();
}

// ─── SETUP ──────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  Serial.println("=================================");
  Serial.println("  PlantMind — Firebase Edition");
  Serial.println("  Heltec LoRa32 V3 (ESP32-S3)");
  Serial.println("  KY-028 Temperature Sensor");
  Serial.println("=================================");

  connectWiFi();

  Serial.println();
  Serial.print("Pump ON < "); Serial.print(MOISTURE_LOW);
  Serial.print("% | Duration: "); Serial.print(PUMP_DURATION / 1000);
  Serial.print("s | Cooldown: "); Serial.print(PUMP_COOLDOWN / 1000);
  Serial.println("s");
  Serial.print("Upload interval: ");
  Serial.print(UPLOAD_INTERVAL / 1000);
  Serial.println(" seconds");
  Serial.println();
}

// ─── MAIN LOOP ──────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Read sensors every 2 seconds ──
  if (now - lastRead >= READ_INTERVAL) {
    lastRead = now;

    // Read temperature from KY-028
    lastTemperature = readTemperature();

    // Read soil moisture (averaged, capacitive — inverted)
    long moistureSum = 0;
    for (int s = 0; s < MOISTURE_SAMPLES; s++) {
      moistureSum += analogRead(MOISTURE_PIN);
      delay(10);
    }
    lastMoisture = 100.0 - (((moistureSum / MOISTURE_SAMPLES) / (float)ADC_MAX) * 100.0);

    // Pump control: trigger a 2-second burst if moisture is low
    if (!pumpRunning && lastMoisture < MOISTURE_LOW
        && (millis() - lastPumpEnd >= PUMP_COOLDOWN)) {
      pumpRunning = true;
      pumpStarted = millis();
      digitalWrite(RELAY_PIN, RELAY_ON);
      pumpStateChanged = true;
      Serial.println(">>> PUMP ON — 2 second burst");
    }

    // Print to serial
    Serial.print("Moisture: ");  Serial.print(lastMoisture, 1);
    Serial.print("% | Temp: ");  Serial.print(lastTemperature, 1);
    Serial.print("°C | Pump: "); Serial.println(pumpRunning ? "ON" : "OFF");
  }

  // ── Safety: auto-stop pump after PUMP_DURATION (runs every loop) ──
  if (pumpRunning && (millis() - pumpStarted >= PUMP_DURATION)) {
    pumpRunning = false;
    digitalWrite(RELAY_PIN, RELAY_OFF);
    lastPumpEnd = millis();
    Serial.println(">>> PUMP OFF — 2 second burst complete");

    // Re-read moisture right after watering to capture the change
    long moistureSum = 0;
    for (int s = 0; s < MOISTURE_SAMPLES; s++) {
      moistureSum += analogRead(MOISTURE_PIN);
      delay(10);
    }
    lastMoisture = 100.0 - (((moistureSum / MOISTURE_SAMPLES) / (float)ADC_MAX) * 100.0);
    Serial.print(">>> Post-water moisture: ");
    Serial.print(lastMoisture, 1);
    Serial.println("%");

    pumpStateChanged = true;
  }

  // ── Upload immediately on pump state change ──
  if (pumpStateChanged) {
    pumpStateChanged = false;
    Serial.println(">>> Pump state changed — uploading now");
    sendToFirebase(lastMoisture, lastTemperature, pumpRunning);
    lastUpload = millis();
  }

  // ── Regular upload every 60 seconds ──
  if (millis() - lastUpload >= UPLOAD_INTERVAL) {
    lastUpload = millis();
    sendToFirebase(lastMoisture, lastTemperature, pumpRunning);
  }
}
