// ============================================================================
// Motorized catheter accumulator firmware for Arduino UNO R3, TB6600, and T8 stage
//
// Verified common-anode wiring:
//   Arduino 5V -> PUL+ / DIR+
//   Arduino D2 -> PUL-
//   Arduino D3 -> DIR-
//   ENA not connected
//
// The optocoupler conducts while PUL- is LOW, so the idle PUL level must be HIGH.
// Motor: 200 full steps/rev * 8 microsteps / 8 mm lead = 200 pulses/mm.
//
// Direction check:
// If Forward in the GUI moves the stage in the wrong physical direction, change
// FORWARD_DIR_LEVEL below from HIGH to LOW and upload the firmware again.
//
// This system has no encoder, home sensor, or physical limit switch. Position is
// estimated only from step pulses that were actually generated. Every power cycle
// or reset starts in UNCALIBRATED; the firmware never assumes the real position is 0.
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

const uint8_t PUL_PIN = 2;
const uint8_t DIR_PIN = 3;

// Change only HIGH to LOW if the physical direction is reversed.
const uint8_t FORWARD_DIR_LEVEL = HIGH;
const uint8_t REVERSE_DIR_LEVEL = (FORWARD_DIR_LEVEL == HIGH) ? LOW : HIGH;

const int MOTOR_STEPS_PER_REV = 200;
const int MICROSTEPS = 8;
const float LEAD_MM_PER_REV = 8.0f;
const float PULSES_PER_MM =
    ((float)MOTOR_STEPS_PER_REV * (float)MICROSTEPS) / LEAD_MM_PER_REV;

const float MIN_POSITION_MM = 0.0f;
const float MAX_POSITION_MM = 280.0f;
const float SOFTWARE_TRAVEL_MM = MAX_POSITION_MM - MIN_POSITION_MM;
const long MIN_POSITION_PULSES = 0L;
const long MAX_POSITION_PULSES = 56000L;  // 280 mm * 200 pulse/mm

const float MIN_SPEED_MM_S = 0.5f;
const float MAX_SPEED_MM_S = 15.0f;
const float DEFAULT_SPEED_MM_S = 5.0f;

const float MIN_ACCEL_MM_S2 = 1.0f;
const float MAX_ACCEL_MM_S2 = 100.0f;
const float DEFAULT_ACCEL_MM_S2 = 20.0f;

const unsigned long DIR_SETUP_US = 100UL;
const unsigned long WATCHDOG_TIMEOUT_MS = 1000UL;
const unsigned long STATUS_INTERVAL_MS = 200UL;
const uint8_t SERIAL_BUFFER_SIZE = 80;

enum MotionState {
  IDLE,
  JOG_FORWARD,
  JOG_REVERSE,
  MOVE_TO_TARGET
};

MotionState motionState = IDLE;
bool calibrated = false;
long currentPositionPulses = 0L;
long targetPositionPulses = 0L;
int8_t motionDirection = 0;  // +1 Forward, -1 Reverse

float commandedSpeedMmS = DEFAULT_SPEED_MM_S;
float commandedAccelMmS2 = DEFAULT_ACCEL_MM_S2;
float currentSpeedPps = 0.0f;

bool pulseIsLow = false;
bool finishMoveAfterPulse = false;
unsigned long nextEdgeUs = 0UL;
unsigned long currentHalfPeriodUs = 500UL;

unsigned long lastHostContactMs = 0UL;
unsigned long lastStatusMs = 0UL;

char serialBuffer[SERIAL_BUFFER_SIZE];
uint8_t serialLength = 0;

const __FlashStringHelper *stateName(MotionState state) {
  switch (state) {
    case JOG_FORWARD: return F("JOG_FWD");
    case JOG_REVERSE: return F("JOG_REV");
    case MOVE_TO_TARGET: return F("MOVE");
    default: return F("IDLE");
  }
}

bool timeReached(unsigned long now, unsigned long deadline) {
  return (long)(now - deadline) >= 0;
}

void printPositionLine() {
  Serial.print(F("POS "));
  if (!calibrated) {
    Serial.println(F("UNCALIBRATED"));
  } else {
    Serial.println(currentPositionPulses / PULSES_PER_MM, 3);
  }
}

void printStateLine() {
  Serial.print(F("STATE "));
  Serial.println(stateName(motionState));
}

void printTargetLine() {
  Serial.print(F("TARGET "));
  if (motionState == MOVE_TO_TARGET) {
    Serial.println(targetPositionPulses / PULSES_PER_MM, 3);
  } else {
    Serial.println(F("--"));
  }
}

void sendStatus() {
  // A status line fits in the UNO TX buffer and is sent every 200 ms during motion.
  Serial.print(F("STATUS "));
  Serial.print(stateName(motionState));
  Serial.print(' ');
  if (calibrated) {
    Serial.print(currentPositionPulses / PULSES_PER_MM, 3);
  } else {
    Serial.print(F("UNCALIBRATED"));
  }
  Serial.print(' ');
  if (motionState == MOVE_TO_TARGET) {
    Serial.println(targetPositionPulses / PULSES_PER_MM, 3);
  } else {
    Serial.println('-');
  }
}

void forcePulseIdle() {
  digitalWrite(PUL_PIN, HIGH);
  pulseIsLow = false;
  finishMoveAfterPulse = false;
}

void stopMotion(bool reportStopped) {
  bool wasMoving = (motionState != IDLE);
  forcePulseIdle();
  motionState = IDLE;
  motionDirection = 0;
  currentSpeedPps = 0.0f;

  if (reportStopped || wasMoving) {
    Serial.println(F("STOPPED"));
  }
  printStateLine();
  printPositionLine();
  printTargetLine();
}

unsigned long halfPeriodForSpeed(float speedPps) {
  if (speedPps < 1.0f) speedPps = 1.0f;
  float halfPeriod = 500000.0f / speedPps;
  if (halfPeriod < 150.0f) halfPeriod = 150.0f;
  if (halfPeriod > 500000.0f) halfPeriod = 500000.0f;
  return (unsigned long)(halfPeriod + 0.5f);
}

float firstPulseSpeedPps() {
  // Speed after the first pulse from rest, derived from v^2 = 2*a*s.
  float accelPps2 = commandedAccelMmS2 * PULSES_PER_MM;
  float speed = sqrtf(2.0f * accelPps2);
  float maximum = commandedSpeedMmS * PULSES_PER_MM;
  return (speed < maximum) ? speed : maximum;
}

void setDirection(int8_t direction) {
  digitalWrite(DIR_PIN, direction > 0 ? FORWARD_DIR_LEVEL : REVERSE_DIR_LEVEL);
}

void beginMotion(MotionState newState, int8_t direction) {
  motionState = newState;
  motionDirection = direction;
  currentSpeedPps = firstPulseSpeedPps();
  currentHalfPeriodUs = halfPeriodForSpeed(currentSpeedPps);
  pulseIsLow = false;
  finishMoveAfterPulse = false;
  setDirection(direction);
  nextEdgeUs = micros() + DIR_SETUP_US;
}

bool nextPulseWouldCrossLimit() {
  if (!calibrated) return false;
  long nextPosition = currentPositionPulses + motionDirection;
  return nextPosition < MIN_POSITION_PULSES ||
         nextPosition > MAX_POSITION_PULSES;
}

void stopAtLimit() {
  bool atMaximum = (motionDirection > 0);
  forcePulseIdle();
  motionState = IDLE;
  motionDirection = 0;
  currentSpeedPps = 0.0f;
  printStateLine();
  printPositionLine();
  printTargetLine();
  Serial.print(F("LIMIT "));
  Serial.println(atMaximum ? F("MAX") : F("MIN"));
}

void finishFixedMove() {
  forcePulseIdle();
  motionState = IDLE;
  motionDirection = 0;
  currentSpeedPps = 0.0f;
  Serial.println(F("DONE"));
  printStateLine();
  printPositionLine();
  printTargetLine();
}

void updateSpeedAfterPulse() {
  float accelPps2 = commandedAccelMmS2 * PULSES_PER_MM;
  float maximumPps = commandedSpeedMmS * PULSES_PER_MM;
  float accelerated = sqrtf(currentSpeedPps * currentSpeedPps + 2.0f * accelPps2);
  if (accelerated > maximumPps) accelerated = maximumPps;

  if (motionState == MOVE_TO_TARGET) {
    long remaining = labs(targetPositionPulses - currentPositionPulses);
    // Ensure the stage can decelerate to zero within the remaining pulses.
    // Short moves naturally produce a triangular velocity profile.
    float stoppingLimited = sqrtf(2.0f * accelPps2 * (float)remaining);
    currentSpeedPps = (accelerated < stoppingLimited) ? accelerated : stoppingLimited;
  } else {
    // Jog accelerates only at startup and stops immediately on release or STOP.
    currentSpeedPps = accelerated;
  }

  if (currentSpeedPps < 1.0f) currentSpeedPps = 1.0f;
  currentHalfPeriodUs = halfPeriodForSpeed(currentSpeedPps);
}

void updatePulseGenerator() {
  if (motionState == IDLE) return;

  unsigned long now = micros();
  if (!timeReached(now, nextEdgeUs)) return;

  if (!pulseIsLow) {
    if (nextPulseWouldCrossLimit()) {
      stopAtLimit();
      return;
    }

    // Common anode: LOW begins the active pulse. Update position only when a
    // pulse is actually generated.
    digitalWrite(PUL_PIN, LOW);
    pulseIsLow = true;
    currentPositionPulses += motionDirection;
    nextEdgeUs = now + currentHalfPeriodUs;

    if (motionState == MOVE_TO_TARGET &&
        currentPositionPulses == targetPositionPulses) {
      finishMoveAfterPulse = true;
    }
  } else {
    digitalWrite(PUL_PIN, HIGH);
    pulseIsLow = false;

    if (finishMoveAfterPulse) {
      finishFixedMove();
      return;
    }

    updateSpeedAfterPulse();
    nextEdgeUs = now + currentHalfPeriodUs;
  }
}

bool parseFiniteFloat(const char *text, float &value) {
  if (text == NULL || *text == '\0') return false;
  char *endPointer;
  double parsed = strtod(text, &endPointer);
  while (*endPointer == ' ' || *endPointer == '\t') endPointer++;
  if (*endPointer != '\0' || isnan(parsed) || isinf(parsed)) return false;
  value = (float)parsed;
  return true;
}

void acknowledgeHostContact() {
  lastHostContactMs = millis();
}

void handleSetSpeed(const char *argument) {
  float value;
  if (!parseFiniteFloat(argument, value) ||
      value < MIN_SPEED_MM_S || value > MAX_SPEED_MM_S) {
    Serial.println(F("ERR INVALID_PARAMETER"));
    return;
  }
  if (motionState != IDLE) {
    Serial.println(F("ERR BUSY"));
    return;
  }
  commandedSpeedMmS = value;
  acknowledgeHostContact();
  Serial.print(F("ACK SPEED "));
  Serial.println(commandedSpeedMmS, 3);
}

void handleSetAccel(const char *argument) {
  float value;
  if (!parseFiniteFloat(argument, value) ||
      value < MIN_ACCEL_MM_S2 || value > MAX_ACCEL_MM_S2) {
    Serial.println(F("ERR INVALID_PARAMETER"));
    return;
  }
  if (motionState != IDLE) {
    Serial.println(F("ERR BUSY"));
    return;
  }
  commandedAccelMmS2 = value;
  acknowledgeHostContact();
  Serial.print(F("ACK ACCEL "));
  Serial.println(commandedAccelMmS2, 3);
}

void handleJog(int8_t direction) {
  if (motionState != IDLE) {
    Serial.println(F("ERR BUSY"));
    return;
  }

  if (calibrated) {
    if (direction > 0 && currentPositionPulses >= MAX_POSITION_PULSES) {
      Serial.println(F("LIMIT MAX"));
      return;
    }
    if (direction < 0 && currentPositionPulses <= MIN_POSITION_PULSES) {
      Serial.println(F("LIMIT MIN"));
      return;
    }
  }

  acknowledgeHostContact();
  beginMotion(direction > 0 ? JOG_FORWARD : JOG_REVERSE, direction);
  Serial.print(F("ACK JOG "));
  Serial.println(direction > 0 ? F("FWD") : F("REV"));
  printStateLine();
  printTargetLine();
}

void handleMove(const char *argument) {
  if (!calibrated) {
    Serial.println(F("ERR NOT_ZEROED"));
    return;
  }
  if (motionState != IDLE) {
    Serial.println(F("ERR BUSY"));
    return;
  }

  float distanceMm;
  if (!parseFiniteFloat(argument, distanceMm) || distanceMm == 0.0f) {
    Serial.println(F("ERR INVALID_PARAMETER"));
    return;
  }

  // A relative move larger than the full software travel is always invalid.
  // Check it first to avoid overflow when converting a large float to long.
  if (fabsf(distanceMm) > SOFTWARE_TRAVEL_MM) {
    Serial.println(F("ERR OUT_OF_RANGE"));
    return;
  }

  long deltaPulses = lroundf(distanceMm * PULSES_PER_MM);
  if (deltaPulses == 0L) {
    Serial.println(F("ERR INVALID_PARAMETER"));
    return;
  }

  long requestedTarget = currentPositionPulses + deltaPulses;
  if (requestedTarget < MIN_POSITION_PULSES ||
      requestedTarget > MAX_POSITION_PULSES) {
    Serial.println(F("ERR OUT_OF_RANGE"));
    return;  // Reject the entire MOVE instead of clipping the distance.
  }

  targetPositionPulses = requestedTarget;
  acknowledgeHostContact();
  beginMotion(MOVE_TO_TARGET, deltaPulses > 0L ? 1 : -1);
  Serial.println(F("ACK MOVE"));
  printTargetLine();
  printStateLine();
}

void handleZero() {
  if (motionState != IDLE) {
    Serial.println(F("ERR BUSY"));
    return;
  }
  currentPositionPulses = 0L;
  targetPositionPulses = 0L;
  calibrated = true;
  acknowledgeHostContact();
  Serial.println(F("ACK ZERO"));
  printPositionLine();
  printStateLine();
  printTargetLine();
}

void handleCommand(char *command) {
  while (*command == ' ' || *command == '\t') command++;
  size_t length = strlen(command);
  while (length > 0 &&
         (command[length - 1] == ' ' || command[length - 1] == '\t')) {
    command[--length] = '\0';
  }

  if (strcmp(command, "HB") == 0) {
    acknowledgeHostContact();
  } else if (strcmp(command, "PING") == 0) {
    acknowledgeHostContact();
    Serial.println(F("PONG"));
  } else if (strcmp(command, "GET STATUS") == 0) {
    acknowledgeHostContact();
    sendStatus();
    Serial.print(F("PARAM SPEED="));
    Serial.print(commandedSpeedMmS, 3);
    Serial.print(F(" ACCEL="));
    Serial.println(commandedAccelMmS2, 3);
  } else if (strcmp(command, "STOP") == 0) {
    acknowledgeHostContact();
    stopMotion(true);
  } else if (strcmp(command, "ZERO") == 0) {
    handleZero();
  } else if (strcmp(command, "JOG FWD") == 0) {
    handleJog(1);
  } else if (strcmp(command, "JOG REV") == 0) {
    handleJog(-1);
  } else if (strncmp(command, "SET SPEED ", 10) == 0) {
    handleSetSpeed(command + 10);
  } else if (strncmp(command, "SET ACCEL ", 10) == 0) {
    handleSetAccel(command + 10);
  } else if (strncmp(command, "MOVE ", 5) == 0) {
    handleMove(command + 5);
  } else {
    Serial.println(F("ERR INVALID_COMMAND"));
  }
}

void readSerialCommands() {
  // Process at most 16 bytes per loop so long serial input cannot monopolize
  // execution and disturb pulse timing.
  uint8_t processed = 0;
  while (Serial.available() > 0 && processed < 16) {
    char incoming = (char)Serial.read();
    processed++;

    if (incoming == '\n') {
      serialBuffer[serialLength] = '\0';
      if (serialLength > 0) handleCommand(serialBuffer);
      serialLength = 0;
    } else if (incoming != '\r') {
      if (serialLength < SERIAL_BUFFER_SIZE - 1) {
        serialBuffer[serialLength++] = incoming;
      } else {
        serialLength = 0;
        Serial.println(F("ERR LINE_TOO_LONG"));
      }
    }
  }
}

void checkCommunicationWatchdog() {
  if (motionState == IDLE) return;
  if ((unsigned long)(millis() - lastHostContactMs) > WATCHDOG_TIMEOUT_MS) {
    stopMotion(false);
    Serial.println(F("ERR LINK_TIMEOUT"));
  }
}

void sendPeriodicStatus() {
  unsigned long now = millis();
  if ((unsigned long)(now - lastStatusMs) >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    sendStatus();
  }
}

void setup() {
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(PUL_PIN, HIGH);  // Common-anode idle level; no motion at startup.
  digitalWrite(DIR_PIN, FORWARD_DIR_LEVEL);

  Serial.begin(115200);
  lastHostContactMs = millis();
  lastStatusMs = millis();

  Serial.println(F("READY"));
  printPositionLine();
  printStateLine();
  printTargetLine();
}

void loop() {
  // Update the pulse generator twice to reduce edge jitter from serial parsing.
  updatePulseGenerator();
  readSerialCommands();
  updatePulseGenerator();
  checkCommunicationWatchdog();
  sendPeriodicStatus();
}
