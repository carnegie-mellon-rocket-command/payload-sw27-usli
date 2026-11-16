#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// Connections use GPIO numbers, not physical header positions.
constexpr int BMP_SDA = 2;
constexpr int BMP_SCL = 3;

constexpr int SD_SCK  = 4;
constexpr int SD_MISO = 5;
constexpr int SD_MOSI = 6;
constexpr int SD_CS   = 7;

constexpr int MOTOR_DIR = 0;
constexpr int MOTOR_PWM = 1;

// PWM: Arduino-ESP32 board package 3.x.
constexpr int PWM_FREQ = 5000;
constexpr int PWM_BITS = 8;
constexpr int MOTOR_DUTY = 200;  // 0–255

constexpr uint32_t MOTOR_RUN_MS = 300000;  // Five minutes
constexpr uint32_t SAMPLE_MS = 100;
constexpr uint32_t FLUSH_MS = 1000;

// Starting values to tune using recorded test data.
constexpr float LAUNCH_HEIGHT_M = 5.0f;
constexpr uint32_t LAUNCH_CONFIRM_MS = 1000;
constexpr float DESCENT_DROP_M = 3.0f;

// Landing requires ten seconds of stable readings.
// This version assumes landing close to launch elevation.
constexpr float GROUND_BAND_M = 2.0f;
constexpr float STABLE_RANGE_M = 0.8f;
constexpr size_t WINDOW_SIZE = 101;
constexpr uint32_t STABLE_TIME_MS = 10000;
constexpr uint32_t MAX_SAMPLE_GAP_MS = 300;

Adafruit_BMP280 bmp;
File logFile;

enum FlightState {
  WAITING,
  ASCENDING,
  DESCENDING,
  LANDED
};

FlightState state = WAITING;

float groundAltitude = 0;
float peakAltitude = 0;

float altitudeWindow[WINDOW_SIZE];
uint32_t timeWindow[WINDOW_SIZE];
size_t windowIndex = 0;
size_t windowCount = 0;

bool launchCandidate = false;
uint32_t launchCandidateStart = 0;

bool motorRunning = false;
uint32_t motorStart = 0;

uint32_t lastSample = 0;
uint32_t lastValidSample = 0;
uint32_t lastFlush = 0;

const char *stateName() {
  switch (state) {
    case WAITING:    return "WAITING";
    case ASCENDING:  return "ASCENDING";
    case DESCENDING: return "DESCENDING";
    case LANDED:    return "LANDED";
  }
  return "UNKNOWN";
}

// Stop initialization if essential hardware is unavailable.
// The motor stays off.
void halt(const char *message) {
  digitalWrite(MOTOR_PWM, LOW);
  Serial.println(message);

  while (true) {
    delay(100);
  }
}

void clearWindow() {
  windowIndex = 0;
  windowCount = 0;
}

void addAltitude(float altitude, uint32_t now) {
  altitudeWindow[windowIndex] = altitude;
  timeWindow[windowIndex] = now;

  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  if (windowCount < WINDOW_SIZE) {
    windowCount++;
  }
}

// Check the whole altitude window instead of only comparing
// two consecutive readings.
bool landingWindowStable(uint32_t now) {
  if (windowCount < WINDOW_SIZE) {
    return false;
  }

  // Once full, windowIndex points to the oldest entry.
  if (now - timeWindow[windowIndex] < STABLE_TIME_MS) {
    return false;
  }

  float lowest = altitudeWindow[0];
  float highest = altitudeWindow[0];

  for (size_t i = 0; i < WINDOW_SIZE; i++) {
    float h = altitudeWindow[i];

    if (fabsf(h) > GROUND_BAND_M) {
      return false;
    }

    lowest = fminf(lowest, h);
    highest = fmaxf(highest, h);
  }

  return (highest - lowest) <= STABLE_RANGE_M;
}

void updateFlight(float altitude, uint32_t now) {
  switch (state) {
    case WAITING:
      if (altitude > LAUNCH_HEIGHT_M) {
        if (!launchCandidate) {
          launchCandidate = true;
          launchCandidateStart = now;
        }

        if (now - launchCandidateStart >= LAUNCH_CONFIRM_MS) {
          state = ASCENDING;
          peakAltitude = altitude;
          Serial.println("Launch detected");
        }
      } else {
        launchCandidate = false;
      }
      break;

    case ASCENDING:
      peakAltitude = fmaxf(peakAltitude, altitude);

      if (peakAltitude - altitude >= DESCENT_DROP_M) {
        state = DESCENDING;
        clearWindow();
        Serial.println("Descent detected");
      }
      break;

    case DESCENDING:
      addAltitude(altitude, now);

      if (landingWindowStable(now)) {
        state = LANDED;

        // Default is detection-only until bench tests pass.
        Serial.println("Landing detected");

        // Uncomment these three lines to enable the motor.
        // motorStart = millis();
        // motorRunning = true;
        // ledcWrite(MOTOR_PWM, MOTOR_DUTY);
      }
      break;

    case LANDED:
      // Remain landed until reset; do not trigger again.
      break;
  }
}

void setup() {
  // Hold the motor control low immediately.
  pinMode(MOTOR_PWM, OUTPUT);
  digitalWrite(MOTOR_PWM, LOW);

  pinMode(MOTOR_DIR, OUTPUT);
  digitalWrite(MOTOR_DIR, LOW);

  Serial.begin(115200);

  // Allow a short USB connection period without waiting forever.
  uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 2000) {
    delay(10);
  }

  if (!ledcAttach(MOTOR_PWM, PWM_FREQ, PWM_BITS)) {
    halt("ERROR: PWM initialization failed");
  }
  ledcWrite(MOTOR_PWM, 0);

  Wire.begin(BMP_SDA, BMP_SCL);

  // BMP280 breakout addresses are commonly 0x76 or 0x77.
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    halt("ERROR: BMP280 not found");
  }

  // Short standby interval allows fresh measurements at 10 Hz.
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X4,
    Adafruit_BMP280::STANDBY_MS_1
  );

  Serial.println("Keep sensor stationary: measuring ground reference");
  delay(1000);

  // Average several starting measurements.
  float altitudeSum = 0;

  for (int i = 0; i < 30; i++) {
    float h = bmp.readAltitude(1013.25f);

    if (!isfinite(h)) {
      halt("ERROR: Invalid ground-reference reading");
    }

    altitudeSum += h;
    delay(SAMPLE_MS);
  }

  groundAltitude = altitudeSum / 30.0f;

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI)) {
    halt("ERROR: SD initialization failed");
  }

  // Create a separate log for each startup.
  char filename[24];
  bool opened = false;

  for (int i = 0; i < 1000; i++) {
    snprintf(filename, sizeof(filename), "/flight%03d.csv", i);

    if (!SD.exists(filename)) {
      logFile = SD.open(filename, FILE_WRITE);
      opened = (bool)logFile;
      break;
    }
  }

  if (!opened) {
    halt("ERROR: Could not create log");
  }

  logFile.println("time_ms,altitude_m,state,motor_running");
  logFile.flush();

  Serial.print("Logging to ");
  Serial.println(filename);
  Serial.println("Ready; motor activation is disabled by default");

  lastSample = millis();
  lastValidSample = lastSample;
  lastFlush = lastSample;
}

void loop() {
  uint32_t now = millis();

  // Check motor timeout independently of the sample schedule.
  if (motorRunning && now - motorStart >= MOTOR_RUN_MS) {
    ledcWrite(MOTOR_PWM, 0);
    motorRunning = false;
    Serial.println("Motor stopped");
  }

  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;

    float altitude =
      bmp.readAltitude(1013.25f) - groundAltitude;

    if (!isfinite(altitude)) {
      clearWindow();
      launchCandidate = false;
      Serial.println("Invalid sensor reading; detection window reset");
      return;
    }

    // Do not treat a long gap in measurements as continuous stability.
    if (now - lastValidSample > MAX_SAMPLE_GAP_MS) {
      clearWindow();
      launchCandidate = false;
    }
    lastValidSample = now;

    updateFlight(altitude, now);

    Serial.printf(
      "%.2f m | %s | motor=%d\n",
      altitude, stateName(), motorRunning
    );

    if (logFile) {
      size_t written = logFile.printf(
        "%lu,%.3f,%s,%d\n",
        (unsigned long)now,
        altitude,
        stateName(),
        motorRunning
      );

      if (written == 0) {
        Serial.println("ERROR: SD write failed");
        logFile.close();
      }
    }
  }

  if (now - lastFlush >= FLUSH_MS) {
    lastFlush = now;

    if (logFile) {
      logFile.flush();
    }
  }
}

