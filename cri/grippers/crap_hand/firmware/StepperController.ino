// Copied from PythonStepper_V7/arduino/StepperController/StepperController.ino
// See original for comments and details.
#include <AccelStepper.h>
#include <EEPROM.h>

#define USE_DRIVER
const int stepPin = 3;     // STEP (pulse)
const int dirPin  = 4;     // DIR
const int enablePin = 5;   // ENA (enable)
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

const long stepsPerRevolution = 1000;
long targetPosition = 0;
long minPosition = 0;
long maxPosition = 3000;

const int zIndexPin = 6;
bool useIndexHoming = false;

float maxSpeed = 5000.0f;
float acceleration = 5000.0f;

struct PersistData { uint32_t magic; long lastPosition; };
const uint32_t PERSIST_MAGIC = 0x53545052; // 'STPR'
const int EEPROM_ADDR = 0;

void savePositionToEEPROM(long pos) {
  PersistData d; d.magic = PERSIST_MAGIC; d.lastPosition = pos;
  EEPROM.put(EEPROM_ADDR, d);
#if defined(ESP32)
  EEPROM.commit();
#endif
}

long loadPositionFromEEPROM(bool *ok) {
  PersistData d; EEPROM.get(EEPROM_ADDR, d);
  if (d.magic == PERSIST_MAGIC) { if (ok) *ok = true; return d.lastPosition; }
  if (ok) *ok = false; return 0;
}

void applyMotionSettings() {
  if (maxSpeed < 1.0f) maxSpeed = 1.0f;
  if (acceleration < 1.0f) acceleration = 1.0f;
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
}

void enableDriver() { digitalWrite(enablePin, LOW); }
void disableDriver() { digitalWrite(enablePin, HIGH); }

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
  enableDriver();
  bool ok = false; long persisted = loadPositionFromEEPROM(&ok);
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
  if (stepper.distanceToGo() == 0) { if (now - lastSaveMs > 250) { savePositionToEEPROM(stepper.currentPosition()); lastSaveMs = now; } }
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
        if (cmd == 'P') { long val = strtol(arg, NULL, 10); if (val < minPosition) val = minPosition; if (val > maxPosition) val = maxPosition; targetPosition = val; stepper.moveTo(targetPosition); Serial.print('<'); Serial.print("OK P "); Serial.print(targetPosition); Serial.print('>'); savePositionToEEPROM(stepper.currentPosition()); }
        else if (cmd == 'V') { float val = strtof(arg, NULL); maxSpeed = val; applyMotionSettings(); Serial.print('<'); Serial.print("OK V "); Serial.print(maxSpeed); Serial.print('>'); }
        else if (cmd == 'A') { float val = strtof(arg, NULL); acceleration = val; applyMotionSettings(); Serial.print('<'); Serial.print("OK A "); Serial.print(acceleration); Serial.print('>'); }
        else if (cmd == 'E') { long val = strtol(arg, NULL, 10); if (val == 0) { disableDriver(); Serial.print('<'); Serial.print("OK E 0"); Serial.print('>'); } else { enableDriver(); Serial.print('<'); Serial.print("OK E 1"); Serial.print('>'); } }
      } else if (cmd == 'H') { stepper.setCurrentPosition(0); targetPosition = 0; stepper.moveTo(targetPosition); savePositionToEEPROM(0); Serial.print('<'); Serial.print("OK H"); Serial.print('>'); }
      else if (cmd == 'L') {
        long minVal = 0, maxVal = 0; char *arg = (char*)&buf[j]; char *end1 = NULL;
        minVal = strtol(arg, &end1, 10);
        if (end1) {
          while (*end1 == ' ' || *end1 == '\t') end1++;
          maxVal = strtol(end1, NULL, 10);
          if (minVal <= maxVal) { minPosition = minVal; maxPosition = maxVal; long cur = stepper.currentPosition(); if (cur < minPosition) { stepper.setCurrentPosition(minPosition); } if (cur > maxPosition) { stepper.setCurrentPosition(maxPosition); } if (targetPosition < minPosition) targetPosition = minPosition; if (targetPosition > maxPosition) targetPosition = maxPosition; stepper.moveTo(targetPosition); Serial.print('<'); Serial.print("OK L "); Serial.print(minPosition); Serial.print(' '); Serial.print(maxPosition); Serial.print('>'); savePositionToEEPROM(stepper.currentPosition()); }
          else { Serial.print('<'); Serial.print("ERR L"); Serial.print('>'); }
        } else { Serial.print('<'); Serial.print("ERR L"); Serial.print('>'); }
      } else if (cmd == 'Z' && (i + 1) < len && toupper(buf[i + 1]) == 'H') { homeUsingIndex(); savePositionToEEPROM(stepper.currentPosition()); Serial.print('<'); Serial.print("OK ZH"); Serial.print('>'); }
      else if (cmd == 'X') { bool ok2 = false; long persisted2 = loadPositionFromEEPROM(&ok2); Serial.print('<'); if (ok2) { Serial.print("MEM "); Serial.print(persisted2); } else { Serial.print("MEM NA"); } Serial.print('>'); }
      else if (cmd == '?') { Serial.print('<'); Serial.print("S "); Serial.print(stepper.currentPosition()); Serial.print(' '); Serial.print(targetPosition); Serial.print(' '); Serial.print(maxSpeed); Serial.print(' '); Serial.print(acceleration); Serial.print(' '); Serial.print(digitalRead(enablePin) == LOW ? 1 : 0); Serial.print(' '); Serial.print(minPosition); Serial.print(' '); Serial.print(maxPosition); Serial.print('>'); }
      else { Serial.print('<'); Serial.print("ERR"); Serial.print('>'); }
      inFrame = false; len = 0;
    } else { if (len < sizeof(buf) - 1) buf[len++] = (char)ch; }
  }
}
