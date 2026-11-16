#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>
#include <math.h>
#include <ESP32Servo.h>

// =====================================================
// 1. PIN CONFIGURATION AND SYSTEM SETTINGS
// =====================================================

// BMP280
constexpr int BMP_SDA = 2;
constexpr int BMP_SCL = 3;

// SD Card
constexpr int SD_SCK  = 4;
constexpr int SD_MISO = 5;
constexpr int SD_MOSI = 6;
constexpr int SD_CS   = 7;

// Timing
constexpr uint32_t SAMPLE_MS = 100;
constexpr uint32_t FLUSH_MS = 1000;

// Flight detection
constexpr float LAUNCH_HEIGHT_M = 5.0f;
constexpr uint32_t LAUNCH_CONFIRM_MS = 1000;
constexpr float DESCENT_DROP_M = 3.0f;

// Landing detection
constexpr float GROUND_BAND_M = 2.0f;
constexpr float STABLE_RANGE_M = 0.8f;
constexpr size_t WINDOW_SIZE = 101;
constexpr uint32_t STABLE_TIME_MS = 10000;
constexpr uint32_t MAX_SAMPLE_GAP_MS = 300;

// =====================================================
// 2. GLOBAL VARIABLES AND SYSTEM OBJECTS
// =====================================================

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

// Landing detection window
float altitudeWindow[WINDOW_SIZE];
uint32_t timeWindow[WINDOW_SIZE];

size_t windowIndex = 0;
size_t windowCount = 0;

// Launch detection
bool launchCandidate = false;
uint32_t launchCandidateStart = 0;

// System timing
uint32_t lastSample = 0;
uint32_t lastValidSample = 0;
uint32_t lastFlush = 0;

// Landing event confirmations
bool landingEventRecorded = false;
bool landingConfirmed = false;

uint32_t landingTime = 0;

// =====================================================
// SERVO CONTROL CONFIGURATION
// =====================================================

constexpr int SERVO_PIN = 8;

constexpr int SERVO_STOP = 1500;
constexpr int SERVO_FORWARD = 1700;
constexpr int SERVO_REVERSE = 1300;

constexpr uint32_t SERVO_RUN_MS = 2000;

constexpr int SERVO_MAX_FORWARD = 2000;
constexpr int SERVO_MAX_REVERSE = 1000;

Servo myServo;

bool servoRunning = false;
uint32_t servoStart = 0;

// =====================================================
// UTILITY AND SYSTEM FUNCTIONS
// =====================================================

// Convert flight state into readable text.
const char *stateName() {
  switch (state) {
    case WAITING:    return "WAITING";
    case ASCENDING:  return "ASCENDING";
    case DESCENDING: return "DESCENDING";
    case LANDED:     return "LANDED";
  }

  return "UNKNOWN";
}

// Stop execution if essential hardware fails.
void halt(const char *message) {
  myServo.writeMicroseconds(SERVO_STOP);
  Serial.println(message);

  while (true) {
    delay(100);
  }
}

// =====================================================
// HARDWARE INITIALIZATION FUNCTIONS
// =====================================================

// Initialize BMP280 sensor.
void initializeBMP() {
  Wire.begin(BMP_SDA, BMP_SCL);

  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    halt("ERROR: BMP280 not found");
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X4,
    Adafruit_BMP280::STANDBY_MS_1
  );
}

// Initialize SD card and create a unique flight log.
void initializeSD(char *filename, size_t filenameSize) {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI)) {
    halt("ERROR: SD initialization failed");
  }

  bool opened = false;

  for (int i = 0; i < 1000; i++) {
    snprintf(
      filename,
      filenameSize,
      "/flight%03d.csv",
      i
    );

    if (!SD.exists(filename)) {
      logFile = SD.open(filename, FILE_WRITE);
      opened = (bool)logFile;
      break;
    }
  }

  if (!opened) {
    halt("ERROR: Could not create log");
  }

  logFile.println("time_ms,altitude_m,state,servoRunning");
  logFile.flush();

  Serial.print("Logging to ");
  Serial.println(filename);
}

// =====================================================
// SENSOR CALIBRATION AND READING FUNCTIONS
// =====================================================

// Measure initial ground altitude.
void calibrateGroundAltitude() {
  Serial.println(
    "Keep sensor stationary: measuring ground reference"
  );

  delay(1000);

  float altitudeSum = 0;

  for (int i = 0; i < 30; i++) {
    float altitude = bmp.readAltitude(1013.25f);

    if (!isfinite(altitude)) {
      halt("ERROR: Invalid ground-reference reading");
    }

    altitudeSum += altitude;
    delay(SAMPLE_MS);
  }

  groundAltitude = altitudeSum / 30.0f;
}

// Read altitude relative to the initial ground reference.
bool readAltitude(float &altitude) {
  altitude = bmp.readAltitude(1013.25f) - groundAltitude;

  return isfinite(altitude);
}

// =====================================================
// SERVO CONTROL FUNCTIONS
// =====================================================

// Initialize servo.
void initializeServo() {
  myServo.setPeriodHertz(50);

  if (!myServo.attach(SERVO_PIN, 500, 2500);) {
    halt("ERROR: Servo initialization failed");
  }

  myServo.writeMicroseconds(SERVO_STOP);
}

// Rotate servo forward.
void servoForward() {
  myServo.writeMicroseconds(SERVO_FORWARD);
  servoRunning = true;
  servoStart = millis();
}

// Rotate servo in reverse.
void servoReverse() {
  myServo.writeMicroseconds(SERVO_REVERSE);
  servoRunning = true;
  servoStart = millis();
}

// Stop servo.
void stopServo() {
  myServo.writeMicroseconds(SERVO_STOP);
  servoRunning = false;
}

// Monitor servo runtime.
void updateServo(uint32_t now) {
  if (servoRunning &&
      now - servoStart >= SERVO_RUN_MS) {
    stopServo();
  }
}

void servoForwardMax() {
  myServo.writeMicroseconds(SERVO_MAX_FORWARD);
}

void servoReverseMax() {
  myServo.writeMicroseconds(SERVO_MAX_REVERSE);
}

// =====================================================
// LANDING DETECTION WINDOW FUNCTIONS
// =====================================================

// Reset landing detection window.
void clearWindow() {
  windowIndex = 0;
  windowCount = 0;
}

// Add a new altitude measurement to the window.
void addAltitude(float altitude, uint32_t now) {
  altitudeWindow[windowIndex] = altitude;
  timeWindow[windowIndex] = now;

  windowIndex = (windowIndex + 1) % WINDOW_SIZE;

  if (windowCount < WINDOW_SIZE) {
    windowCount++;
  }
}

// Check whether all altitude readings are stable.
bool landingWindowStable(uint32_t now) {
  if (windowCount < WINDOW_SIZE) {
    return false;
  }

  // windowIndex points to the oldest reading.
  if (now - timeWindow[windowIndex] < STABLE_TIME_MS) {
    return false;
  }

  float lowest = altitudeWindow[0];
  float highest = altitudeWindow[0];

  for (size_t i = 0; i < WINDOW_SIZE; i++) {
    float altitude = altitudeWindow[i];

    if (fabsf(altitude) > GROUND_BAND_M) {
      return false;
    }

    lowest = fminf(lowest, altitude);
    highest = fmaxf(highest, altitude);
  }

  return (highest - lowest) <= STABLE_RANGE_M;
}

// =====================================================
// LANDING EVENT FUNCTIONS
// =====================================================

void recordLandingEvent(uint32_t now) {
  // Report landing.
  Serial.println("========================");
  Serial.println("LANDING CONFIRMED");
  Serial.println("System status: LANDED");
  Serial.printf("Landing time: %lu ms\n",
                (unsigned long)now);
  Serial.println("========================");

  // Record landing event.
  if (logFile) {
    logFile.printf(
      "%lu,%.3f,%s,%d\n",
      (unsigned long)now,
      0.0f,
      "LANDING_EVENT",
      servoRunning
    );

    logFile.flush();
  }
}

void fireMatch();
void spinServo();

void landingEvent(uint32_t now) {

  // Prevent duplicate landing events.
  if (landingEventRecorded) {
    return;
  }

  // Update system status.
  landingConfirmed = true;
  landingEventRecorded = true;
  landingTime = now;

  recordLandingEvent(now);

  // Post-landing actions can be coordinated here.
  fireMatch();
  spinServo();
}

// =====================================================
// FLIGHT STATE DETECTION FUNCTIONS
// =====================================================

// Detect launch using a sustained altitude increase.
void detectLaunch(float altitude, uint32_t now) {
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
}

// Track maximum altitude and detect descent.
void detectDescent(float altitude) {
  peakAltitude = fmaxf(peakAltitude, altitude);

  if (peakAltitude - altitude >= DESCENT_DROP_M) {
    state = DESCENDING;

    clearWindow();

    Serial.println("Descent detected");
  }
}

// Detect landing using a stable altitude window.
void detectLanding(float altitude, uint32_t now) {
  addAltitude(altitude, now);

  if (landingWindowStable(now)) {
    state = LANDED;

    Serial.println("Landing detected");

    landingEvent(now);
  }
}

// Coordinate flight-state detection.
void updateFlight(float altitude, uint32_t now) {
  switch (state) {

    case WAITING:
      detectLaunch(altitude, now);
      break;

    case ASCENDING:
      detectDescent(altitude);
      break;

    case DESCENDING:
      detectLanding(altitude, now);
      break;

    case LANDED:
      // Remain landed until reset.
      break;
  }
}

// =====================================================
// DATA LOGGING FUNCTIONS
// =====================================================

// Write current flight information to the SD card.
void writeLog(uint32_t now, float altitude) {
  if (!logFile) {
    return;
  }

  size_t written = logFile.printf(
    "%lu,%.3f,%s,%d\n",
    (unsigned long)now,
    altitude,
    stateName(),
    servoRunning
  );

  if (written == 0) {
    Serial.println("ERROR: SD write failed");
    logFile.close();
  }
}

// Periodically flush buffered data to the SD card.
void flushLog() {
  if (logFile) {
    logFile.flush();
  }
}

// =====================================================
// 10. SENSOR PROCESSING AND MONITORING
// =====================================================

// Process a new sensor reading and update flight state.
bool processSample(uint32_t now) {
  float altitude;

  if (!readAltitude(altitude)) {
    clearWindow();
    launchCandidate = false;

    Serial.println(
      "Invalid sensor reading; detection window reset"
    );

    return false;
  }

  // Reset detection window after an excessive sample gap.
  if (now - lastValidSample > MAX_SAMPLE_GAP_MS) {
    clearWindow();
    launchCandidate = false;
  }

  lastValidSample = now;

  updateFlight(altitude, now);

  // Display current flight information.
  Serial.printf(
    "%.2f m | %s | servo=%d\n",
    altitude,
    stateName(),
    servoRunning
  );

  // Save current measurement.
  writeLog(now, altitude);

  return true;
}

// =====================================================
// ARDUINO SETUP
// =====================================================

void setup() {

  // Initialize serial communication.
  Serial.begin(115200);

  // Allow time for USB serial connection.
  uint32_t serialStart = millis();

  while (!Serial && millis() - serialStart < 2000) {
    delay(10);
  }

  // Initialize hardware.
  initializeServo();
  initializeBMP();

  // Establish ground altitude reference.
  calibrateGroundAltitude();

  // Initialize SD card and create log.
  char filename[24];

  initializeSD(filename, sizeof(filename));

  Serial.println(
    "Ready"
  );

  // Initialize system timers.
  lastSample = millis();
  lastValidSample = lastSample;
  lastFlush = lastSample;
}

// =====================================================
// ARDUINO MAIN LOOP
// =====================================================

void loop() {
  uint32_t now = millis();

  // Monitor servo.
  updateServo(now);

  // Process sensor data at the specified interval.
  if (now - lastSample >= SAMPLE_MS) {
    lastSample = now;

    if (!processSample(now)) {
      return;
    }
  }

  // Periodically save buffered SD-card data.
  if (now - lastFlush >= FLUSH_MS) {
    lastFlush = now;

    flushLog();
  }
}
