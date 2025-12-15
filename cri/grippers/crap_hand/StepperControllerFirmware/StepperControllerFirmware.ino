// Copied from PythonStepper_V7/arduino/StepperController/StepperController.ino
// See original for comments and details.
#include <AccelStepper.h>
#include <EEPROM.h>
#include <math.h>

#define USE_DRIVER
const int stepPin = 3;     // STEP (pulse)
const int dirPin  = 4;     // DIR
const int enablePin = 5;   // ENA (enable)
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

const long stepsPerRevolution = 1000;
long targetPosition = 0;
long minPosition = 0;
long maxPosition = 3000;

// Hardcoded limits that override any saved values on boot
const long HARD_MIN_POSITION = 0;
const long HARD_MAX_POSITION = 3000;

const int zIndexPin = 6;
bool useIndexHoming = false;

float maxSpeed = 5000.0f;
float acceleration = 5000.0f;
static bool autoDisablePending = false; // auto-disable driver after current move completes

struct PersistData {
  uint32_t magic;
  long lastPosition;
  float maxSpeed;
  float acceleration;
  long minPosition;
  long maxPosition;
};
const uint32_t PERSIST_MAGIC = 0x53545032; // 'STP2' (bump to invalidate old layout)
const int EEPROM_ADDR = 0;
static PersistData persistedCache;

void saveAllToEEPROMIfChanged(const PersistData &d) {
  if (persistedCache.magic == d.magic &&
      persistedCache.lastPosition == d.lastPosition &&
      persistedCache.maxSpeed == d.maxSpeed &&
      persistedCache.acceleration == d.acceleration &&
      persistedCache.minPosition == d.minPosition &&
      persistedCache.maxPosition == d.maxPosition) {
    return; // no change, skip write
  }
  EEPROM.put(EEPROM_ADDR, d);
#if defined(ESP32)
  EEPROM.commit();
#endif
  persistedCache = d;
}

void savePositionToEEPROM(long pos) {
  PersistData d = persistedCache;
  d.magic = PERSIST_MAGIC;
  d.lastPosition = pos;
  saveAllToEEPROMIfChanged(d);
}

long loadPositionFromEEPROM(bool *ok) {
  PersistData d; EEPROM.get(EEPROM_ADDR, d);
  if (d.magic == PERSIST_MAGIC) {
    if (ok) *ok = true;
    persistedCache = d;
    // Load settings from EEPROM
    maxSpeed = d.maxSpeed;
    acceleration = d.acceleration;
    minPosition = d.minPosition;
    maxPosition = d.maxPosition;
    return d.lastPosition;
  }
  if (ok) *ok = false;
  // initialize cache with current defaults
  persistedCache.magic = PERSIST_MAGIC;
  persistedCache.lastPosition = 0;
  persistedCache.maxSpeed = maxSpeed;
  persistedCache.acceleration = acceleration;
  persistedCache.minPosition = minPosition;
  persistedCache.maxPosition = maxPosition;
  return 0;
}

void applyMotionSettings() {
  if (!isfinite(maxSpeed) || maxSpeed < 1.0f) maxSpeed = 1.0f;
  if (!isfinite(acceleration) || acceleration < 1.0f) acceleration = 1.0f;
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
}

void enableDriver() { digitalWrite(enablePin, LOW); }
void disableDriver() { digitalWrite(enablePin, HIGH); }
bool isDriverEnabled() { return digitalRead(enablePin) == LOW; }

void setup() {
#if defined(ESP32)
  EEPROM.begin(sizeof(PersistData));
#endif

  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  Serial.begin(115200);
  while (!Serial) { ; }

  applyMotionSettings();
  // Start with driver disabled by default
  disableDriver();
  bool ok = false; long persisted = loadPositionFromEEPROM(&ok);
  // Enforce hardcoded limits on every reboot, overwriting any persisted limits
  minPosition = HARD_MIN_POSITION;
  maxPosition = HARD_MAX_POSITION;
  // Persist enforced limits back to EEPROM if they differ
  {
    PersistData d = persistedCache;
    d.magic = PERSIST_MAGIC;
    d.lastPosition = ok ? persisted : 0;
    d.maxSpeed = maxSpeed;
    d.acceleration = acceleration;
    d.minPosition = minPosition;
    d.maxPosition = maxPosition;
    saveAllToEEPROMIfChanged(d);
  }
  // Ensure motion settings applied after potentially loading from EEPROM
  applyMotionSettings();
  long startPos = ok ? persisted : 0;
  if (startPos < minPosition) startPos = minPosition;
  if (startPos > maxPosition) startPos = maxPosition;
  stepper.setCurrentPosition(startPos);
  targetPosition = startPos; stepper.moveTo(targetPosition);

  Serial.println(F("<ready>"));
}

void loop() {
  handleSerial();
  long cur = stepper.currentPosition();
  if (targetPosition < minPosition) targetPosition = minPosition;
  if (targetPosition > maxPosition) targetPosition = maxPosition;
  if (cur <= minPosition && targetPosition < minPosition) targetPosition = minPosition;
  if (cur >= maxPosition && targetPosition > maxPosition) targetPosition = maxPosition;
  stepper.moveTo(targetPosition);
  stepper.run();
  static unsigned long lastSaveMs = 0; unsigned long now = millis();
  if (stepper.distanceToGo() == 0) {
    if (now - lastSaveMs > 250) { savePositionToEEPROM(stepper.currentPosition()); lastSaveMs = now; }
    // If a move-triggering command requested auto-disable, and we're now idle, disable driver
    if (autoDisablePending && isDriverEnabled()) { disableDriver(); autoDisablePending = false; }
  }
  else if (now - lastSaveMs > 2000) { savePositionToEEPROM(stepper.currentPosition()); lastSaveMs = now; }
}

void homeUsingIndex() {
  if (!useIndexHoming) { Serial.println(F("Index homing disabled")); return; }
  pinMode(zIndexPin, INPUT_PULLUP);
  float oldMax = maxSpeed; float oldAcc = acceleration;
  float searchSpeed = 400.0f; float searchAcc = 800.0f;
  maxSpeed = searchSpeed; acceleration = searchAcc; applyMotionSettings();
  bool lastState = digitalRead(zIndexPin);
  unsigned long startMs = millis(); const unsigned long timeoutMs = 8000; long chunk = stepsPerRevolution;
  while (millis() - startMs < timeoutMs) {
    stepper.moveTo(stepper.currentPosition() + chunk);
    while (stepper.distanceToGo() != 0) {
      stepper.run(); bool s = digitalRead(zIndexPin);
      if (s != lastState) {
        lastState = s; stepper.stop(); stepper.run(); stepper.setCurrentPosition(0);
        targetPosition = 0; stepper.moveTo(0); Serial.println(F("Index found; homed to 0"));
        maxSpeed = oldMax; acceleration = oldAcc; applyMotionSettings(); return;
      }
    }
  }
  maxSpeed = oldMax; acceleration = oldAcc; applyMotionSettings(); Serial.println(F("Index not found within timeout"));
}

void handleSerial() {
  static char buf[64]; static size_t len = 0; static bool inFrame = false;
  while (Serial.available()) {
    int ch = Serial.read(); if (ch < 0) break;
    if (!inFrame) { if (ch == '<') { inFrame = true; len = 0; } continue; }
    if (ch == '>') {
      buf[len] = '\0'; size_t i = 0; while (i < len && (buf[i] == ' ' || buf[i] == '\t')) i++;
      char cmd = toupper(buf[i]); size_t j = i + 1; while (j < len && (buf[j] == ' ' || buf[j] == '\t')) j++;
      if (cmd == 'P' || cmd == 'V' || cmd == 'A' || cmd == 'E') {
        char *arg = (char*)&buf[j];
        if (cmd == 'P') {
          enableDriver();
          long val = strtol(arg, NULL, 10); if (val < minPosition) val = minPosition; if (val > maxPosition) val = maxPosition; targetPosition = val; stepper.moveTo(targetPosition);
          // Auto-disable if no motion needed; otherwise defer until move completes
          if (stepper.distanceToGo() != 0) { autoDisablePending = true; }
          else { disableDriver(); }
          Serial.print('<'); Serial.print("OK P "); Serial.print(targetPosition); Serial.print('>'); savePositionToEEPROM(stepper.currentPosition());
        }
        else if (cmd == 'V') { float val = strtof(arg, NULL); maxSpeed = val; applyMotionSettings();
          // Enable for the operation if disabled; disable after if idle and not pending a move
          bool wasEnabled = isDriverEnabled(); if (!wasEnabled) enableDriver();
          PersistData d = persistedCache; d.magic = PERSIST_MAGIC; d.lastPosition = stepper.currentPosition(); d.maxSpeed = maxSpeed; d.acceleration = acceleration; d.minPosition = minPosition; d.maxPosition = maxPosition; saveAllToEEPROMIfChanged(d);
          Serial.print('<'); Serial.print("OK V "); Serial.print(maxSpeed); Serial.print('>');
          if (!wasEnabled && stepper.distanceToGo() == 0 && !autoDisablePending) disableDriver();
        }
        else if (cmd == 'A') { float val = strtof(arg, NULL); acceleration = val; applyMotionSettings();
          bool wasEnabled = isDriverEnabled(); if (!wasEnabled) enableDriver();
          PersistData d = persistedCache; d.magic = PERSIST_MAGIC; d.lastPosition = stepper.currentPosition(); d.maxSpeed = maxSpeed; d.acceleration = acceleration; d.minPosition = minPosition; d.maxPosition = maxPosition; saveAllToEEPROMIfChanged(d);
          Serial.print('<'); Serial.print("OK A "); Serial.print(acceleration); Serial.print('>');
          if (!wasEnabled && stepper.distanceToGo() == 0 && !autoDisablePending) disableDriver();
        }
        else if (cmd == 'E') { long val = strtol(arg, NULL, 10); if (val == 0) { disableDriver(); Serial.print('<'); Serial.print("OK E 0"); Serial.print('>'); } else { enableDriver(); Serial.print('<'); Serial.print("OK E 1"); Serial.print('>'); } }
      } else if (cmd == 'H') {
        enableDriver();
        stepper.setCurrentPosition(0); targetPosition = 0; stepper.moveTo(targetPosition);
        if (stepper.distanceToGo() != 0) { autoDisablePending = true; }
        else { disableDriver(); }
        savePositionToEEPROM(0); Serial.print('<'); Serial.print("OK H"); Serial.print('>');
      }
      else if (cmd == 'L') {
        long minVal = 0, maxVal = 0; char *arg = (char*)&buf[j]; char *end1 = NULL;
        minVal = strtol(arg, &end1, 10);
        if (end1) {
          while (*end1 == ' ' || *end1 == '\t') end1++;
          maxVal = strtol(end1, NULL, 10);
          if (minVal <= maxVal) {
            bool wasEnabled = isDriverEnabled(); if (!wasEnabled) enableDriver();
            minPosition = minVal; maxPosition = maxVal; long cur = stepper.currentPosition(); if (cur < minPosition) { stepper.setCurrentPosition(minPosition); } if (cur > maxPosition) { stepper.setCurrentPosition(maxPosition); } if (targetPosition < minPosition) targetPosition = minPosition; if (targetPosition > maxPosition) targetPosition = maxPosition; stepper.moveTo(targetPosition); Serial.print('<'); Serial.print("OK L "); Serial.print(minPosition); Serial.print(' '); Serial.print(maxPosition); Serial.print('>');
            PersistData d = persistedCache; d.magic = PERSIST_MAGIC; d.lastPosition = stepper.currentPosition(); d.maxSpeed = maxSpeed; d.acceleration = acceleration; d.minPosition = minPosition; d.maxPosition = maxPosition; saveAllToEEPROMIfChanged(d);
            if (!wasEnabled) {
              if (stepper.distanceToGo() == 0 && !autoDisablePending) disableDriver();
              else autoDisablePending = true; // limits change triggered a move; disable after it completes
            }
          }
          else { Serial.print('<'); Serial.print("ERR L"); Serial.print('>'); }
        } else { Serial.print('<'); Serial.print("ERR L"); Serial.print('>'); }
      } else if (cmd == 'Z' && (i + 1) < len && toupper(buf[i + 1]) == 'H') {
        enableDriver();
        homeUsingIndex();
        savePositionToEEPROM(stepper.currentPosition());
        disableDriver();
        Serial.print('<'); Serial.print("OK ZH"); Serial.print('>');
      }
      else if (cmd == 'X') { bool ok2 = false; long persisted2 = loadPositionFromEEPROM(&ok2); Serial.print('<'); if (ok2) { Serial.print("MEM "); Serial.print(persisted2); } else { Serial.print("MEM NA"); } Serial.print('>'); }
      else if (cmd == '?') { Serial.print('<'); Serial.print("S "); Serial.print(stepper.currentPosition()); Serial.print(' '); Serial.print(targetPosition); Serial.print(' '); Serial.print(maxSpeed); Serial.print(' '); Serial.print(acceleration); Serial.print(' '); Serial.print(digitalRead(enablePin) == LOW ? 1 : 0); Serial.print(' '); Serial.print(minPosition); Serial.print(' '); Serial.print(maxPosition); Serial.print('>'); }
      else { Serial.print('<'); Serial.print("ERR"); Serial.print('>'); }
      inFrame = false; len = 0;
    } else { if (len < sizeof(buf) - 1) buf[len++] = (char)ch; }
  }
}
