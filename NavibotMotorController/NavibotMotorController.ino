#include <Arduino.h>
#include <math.h>

// Two independent RPM loops for a classic Arduino Nano (ATmega328P),
// L298N dual H-bridge, and one rising-edge encoder input per motor.
// Direction is commanded; the single encoder input measures speed magnitude.

// ----------------------------- Pin mapping ------------------------------
// Left pins are preserved from the supplied RPM_PID_Control_V1R3.ino.
const uint8_t LEFT_EN  = 10;  // Original ENA PWM
const uint8_t LEFT_IN1 = A4;  // Original IN1
const uint8_t LEFT_IN2 = A3;  // Original IN2
const uint8_t LEFT_ENC = 2;   // Original encoder input, INT0

// Added right motor pins.
const uint8_t RIGHT_EN  = 6;  // ENB PWM
const uint8_t RIGHT_IN1 = 7;  // IN3
const uint8_t RIGHT_IN2 = 8;  // IN4
const uint8_t RIGHT_ENC = 3;  // Encoder input, INT1

// Set to measured rising edges per output-shaft revolution.
// 1000 is provisional and inherited from the supplied sketch.
const float LEFT_COUNTS_PER_REV  = 1000.0f;
const float RIGHT_COUNTS_PER_REV = 1000.0f;

// ------------------------- Protocol assumptions ------------------------
const uint8_t PROTOCOL_VERSION = 0x01;
// Set NAVIBOT_NODE_ID to 0x02 when compiling firmware for the second board.
// Each RS-485 controller must use a unique ID to avoid reply collisions.
#ifndef NAVIBOT_NODE_ID
#define NAVIBOT_NODE_ID 0x01
#endif
const uint8_t NODE_ID = NAVIBOT_NODE_ID;
const uint8_t MSG_REQUEST = 0x01;
const uint8_t MSG_REPLY = 0x81;
const uint8_t SYNC_BYTE = 0x5A;

// Provisional request payload: left RPM (signed int16 LE), right RPM
// (signed int16 LE), in whole RPM. Reply payload: status, reason, left RPM
// (signed int16 LE), right RPM (signed int16 LE). ACK=0, NACK=1. blah
const uint8_t REQUEST_PAYLOAD_LENGTH = 4;
const uint8_t REPLY_PAYLOAD_LENGTH = 6;
const uint8_t MAX_PAYLOAD_LENGTH = 16;
const uint8_t ACK_STATUS = 0;
const uint8_t NACK_STATUS = 1;

enum NackReason : uint8_t {
  NACK_BAD_CRC = 1,
  NACK_BAD_VERSION = 2,
  NACK_BAD_LENGTH = 3,
  NACK_BAD_MESSAGE_TYPE = 4,
  NACK_RPM_OUT_OF_RANGE = 5,
  NACK_FAULT_LATCHED = 6
};

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refin=false, refout=false,
// xorout=0. Provisional coverage is bytes 2 through the end of payload
// (version, node, type, sequence, length, payload); sync and CRC are excluded.
// Wire order is low CRC byte first.

// ------------------------------ Control ---------------------------------
const float MAX_TARGET_RPM = 135.0f;
const float START_LEFT_RPM = 0.0f;
const float START_RIGHT_RPM = 0.0f;
const unsigned long SAMPLE_PERIOD_MS = 100;
const unsigned long COMMAND_TIMEOUT_MS = 1000;  // provisional; tune to CONOPS
const unsigned long STALL_TIMEOUT_MS = 2000;
const uint8_t PWM_MAX = 255;

// Start P-only; tune under restrained, low-risk conditions.
const float KP = 1.70f;
const float KI = 0.05f;
const float KD = 0.0f;
const float INTEGRAL_LIMIT = 150.0f;

// RS-485 direction pin is intentionally not assigned yet. Set this to the
// transceiver's DE/RE control pin when known. 0xFF leaves transmission logic
// active at UART level but does not operate a direction-enable GPIO.
const uint8_t RS485_DIRECTION_PIN = 0xFF;
const uint8_t RS485_TX_ACTIVE_LEVEL = HIGH;

enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_LEFT_ENCODER_STALL,
  FAULT_RIGHT_ENCODER_STALL
};

struct MotorState {
  uint8_t enablePin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  float countsPerRev;
  float targetRPM;
  float measuredRPM;
  float integral;
  float previousError;
  int8_t appliedDirection;
  int8_t reportDirection;
  unsigned long lastPulseMs;
  int pwm;
};

MotorState leftMotor = {
  LEFT_EN, LEFT_IN1, LEFT_IN2, LEFT_COUNTS_PER_REV,
  0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0
};
MotorState rightMotor = {
  RIGHT_EN, RIGHT_IN1, RIGHT_IN2, RIGHT_COUNTS_PER_REV,
  0.0f, 0.0f, 0.0f, 0.0f, 0, 0, 0, 0
};

volatile uint32_t leftEncoderCount = 0;
volatile uint32_t rightEncoderCount = 0;
FaultCode fault = FAULT_NONE;
bool commandTimedOut = false;
unsigned long lastValidCommandMs = 0;
unsigned long previousSampleMs = 0;

// Sequence is echoed in each reply. Store the next expected value in memory;
// sequence rejection is not enabled until the sender's retry policy is known.
uint8_t nextExpectedSequence = 0;
bool sequenceInitialized = false;

// RX frame buffer includes sync, header, payload, and CRC.
uint8_t rxFrame[MAX_PAYLOAD_LENGTH + 9];
uint8_t rxIndex = 0;
uint8_t expectedFrameLength = 0;

void leftEncoderISR() { ++leftEncoderCount; }
void rightEncoderISR() { ++rightEncoderCount; }

uint32_t takeEncoderCount(volatile uint32_t &counter) {
  noInterrupts();
  const uint32_t count = counter;
  counter = 0;
  interrupts();
  return count;
}

void resetPID(MotorState &motor) {
  motor.integral = 0.0f;
  motor.previousError = 0.0f;
}

void stopMotor(MotorState &motor) {
  analogWrite(motor.enablePin, 0);
  digitalWrite(motor.in1Pin, LOW);
  digitalWrite(motor.in2Pin, LOW);
  motor.pwm = 0;
  motor.appliedDirection = 0;
}

void stopAllMotors() {
  stopMotor(leftMotor);
  stopMotor(rightMotor);
}

bool setTargetRPM(uint8_t motorIndex, float rpm) {
  if (motorIndex > 1 || !isfinite(rpm) || fabsf(rpm) > MAX_TARGET_RPM) {
    return false;
  }
  MotorState &motor = (motorIndex == 0) ? leftMotor : rightMotor;
  motor.targetRPM = rpm;
  resetPID(motor);
  motor.lastPulseMs = millis();
  if (rpm == 0.0f) stopMotor(motor);
  return true;
}

bool clearFault() {
  if (leftMotor.targetRPM != 0.0f || rightMotor.targetRPM != 0.0f) return false;
  fault = FAULT_NONE;
  leftMotor.lastPulseMs = millis();
  rightMotor.lastPulseMs = millis();
  resetPID(leftMotor);
  resetPID(rightMotor);
  return true;
}

void latchFault(FaultCode code) {
  if (fault == FAULT_NONE) fault = code;
  stopAllMotors();
}

void applyDirection(MotorState &motor, int8_t direction) {
  if (direction > 0) {
    digitalWrite(motor.in1Pin, HIGH);
    digitalWrite(motor.in2Pin, LOW);
  } else {
    digitalWrite(motor.in1Pin, LOW);
    digitalWrite(motor.in2Pin, HIGH);
  }
  motor.appliedDirection = direction;
  motor.reportDirection = direction;
}

void updateMotor(MotorState &motor,
                 volatile uint32_t &encoderCounter,
                 unsigned long now,
                 float dtSeconds,
                 FaultCode motorStallFault) {
  const uint32_t pulses = takeEncoderCount(encoderCounter);
  motor.measuredRPM = ((float)pulses * 60.0f) /
                      (dtSeconds * motor.countsPerRev);

  if (pulses > 0) {
    motor.lastPulseMs = now;
  } else if (motor.targetRPM != 0.0f &&
             now - motor.lastPulseMs >= STALL_TIMEOUT_MS) {
    latchFault(motorStallFault);
    return;
  }
  if (fault != FAULT_NONE) return;

  if (motor.targetRPM == 0.0f) {
    stopMotor(motor);
    resetPID(motor);
    return;
  }

  const int8_t requestedDirection = (motor.targetRPM > 0.0f) ? 1 : -1;
  // Coast for one control interval before reversing bridge polarity.
  if (motor.appliedDirection != 0 &&
      motor.appliedDirection != requestedDirection) {
    stopMotor(motor);
    resetPID(motor);
    return;
  }
  if (motor.appliedDirection == 0) applyDirection(motor, requestedDirection);

  const float error = fabsf(motor.targetRPM) - motor.measuredRPM;
  const float proposedIntegral = constrain(
      motor.integral + error * dtSeconds, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  const float derivative = (error - motor.previousError) / dtSeconds;
  float output = KP * error + KI * proposedIntegral + KD * derivative;

  const bool drivesFurtherIntoSaturation =
      (output > PWM_MAX && error > 0.0f) || (output < 0.0f && error < 0.0f);
  if (!drivesFurtherIntoSaturation) {
    motor.integral = proposedIntegral;
  } else {
    output = KP * error + KI * motor.integral + KD * derivative;
  }
  motor.previousError = error;
  motor.pwm = (int)constrain(output, 0.0f, (float)PWM_MAX);
  analogWrite(motor.enablePin, motor.pwm);
}

uint16_t calculateCRC(const uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

int16_t readInt16LE(const uint8_t *data) {
  const uint16_t raw = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
  return (int16_t)raw;
}

int16_t measuredSignedRPM(const MotorState &motor) {
  const float signedRPM = motor.measuredRPM * motor.reportDirection;
  if (signedRPM > 32767.0f) return 32767;
  if (signedRPM < -32768.0f) return -32768;
  return (int16_t)lroundf(signedRPM);
}

void writeInt16LE(uint8_t *data, int16_t value) {
  const uint16_t raw = (uint16_t)value;
  data[0] = (uint8_t)(raw & 0xFF);
  data[1] = (uint8_t)(raw >> 8);
}

void sendReply(uint8_t sequence, uint8_t status, uint8_t reason) {
  uint8_t frame[7 + REPLY_PAYLOAD_LENGTH + 2];
  frame[0] = SYNC_BYTE;
  frame[1] = SYNC_BYTE;
  frame[2] = PROTOCOL_VERSION;
  frame[3] = NODE_ID;
  frame[4] = MSG_REPLY;
  frame[5] = sequence;
  frame[6] = REPLY_PAYLOAD_LENGTH;
  frame[7] = status;
  frame[8] = reason;
  writeInt16LE(&frame[9], measuredSignedRPM(leftMotor));
  writeInt16LE(&frame[11], measuredSignedRPM(rightMotor));
  const uint16_t crc = calculateCRC(&frame[2], 5 + REPLY_PAYLOAD_LENGTH);
  frame[13] = (uint8_t)(crc & 0xFF);
  frame[14] = (uint8_t)(crc >> 8);

  if (RS485_DIRECTION_PIN != 0xFF) {
    digitalWrite(RS485_DIRECTION_PIN, RS485_TX_ACTIVE_LEVEL);
    delayMicroseconds(20);
  }
  Serial.write(frame, sizeof(frame));
  Serial.flush();
  if (RS485_DIRECTION_PIN != 0xFF) {
    digitalWrite(RS485_DIRECTION_PIN, !RS485_TX_ACTIVE_LEVEL);
  }
}

void rememberSequence(uint8_t sequence) {
  nextExpectedSequence = (uint8_t)(sequence + 1);
  sequenceInitialized = true;
}

void handleFrame(uint8_t frameLength) {
  const uint8_t node = rxFrame[3];
  const uint8_t messageType = rxFrame[4];
  const uint8_t sequence = rxFrame[5];
  const uint8_t payloadLength = rxFrame[6];

  // Other nodes must remain silent to avoid RS-485 bus collisions.
  // Reply frames are ignored to prevent responses to responses or UART echo.
  if (node != NODE_ID || messageType == MSG_REPLY) return;
  rememberSequence(sequence);

  const uint16_t receivedCRC = (uint16_t)rxFrame[frameLength - 2] |
                               ((uint16_t)rxFrame[frameLength - 1] << 8);
  const uint16_t calculatedCRC = calculateCRC(&rxFrame[2], 5 + payloadLength);
  if (receivedCRC != calculatedCRC) {
    sendReply(sequence, NACK_STATUS, NACK_BAD_CRC);
    return;
  }
  if (rxFrame[2] != PROTOCOL_VERSION) {
    sendReply(sequence, NACK_STATUS, NACK_BAD_VERSION);
    return;
  }
  if (messageType != MSG_REQUEST) {
    sendReply(sequence, NACK_STATUS, NACK_BAD_MESSAGE_TYPE);
    return;
  }
  if (payloadLength != REQUEST_PAYLOAD_LENGTH) {
    sendReply(sequence, NACK_STATUS, NACK_BAD_LENGTH);
    return;
  }

  const int16_t leftRPM = readInt16LE(&rxFrame[7]);
  const int16_t rightRPM = readInt16LE(&rxFrame[9]);
  if (fabsf((float)leftRPM) > MAX_TARGET_RPM ||
      fabsf((float)rightRPM) > MAX_TARGET_RPM) {
    sendReply(sequence, NACK_STATUS, NACK_RPM_OUT_OF_RANGE);
    return;
  }

  // A latched stall fault can be acknowledged/reset only by an explicit
  // all-zero command. A nonzero command cannot restart into the fault.
  if (fault != FAULT_NONE && (leftRPM != 0 || rightRPM != 0)) {
    sendReply(sequence, NACK_STATUS, NACK_FAULT_LATCHED);
    return;
  }

  // Only a fully valid, addressed setpoint refreshes the command watchdog.
  setTargetRPM(0, (float)leftRPM);
  setTargetRPM(1, (float)rightRPM);
  if (fault != FAULT_NONE) clearFault();
  lastValidCommandMs = millis();
  commandTimedOut = false;
  sendReply(sequence, ACK_STATUS, 0);
}

void resetParser(uint8_t lastByte) {
  rxIndex = (lastByte == SYNC_BYTE) ? 1 : 0;
  expectedFrameLength = 0;
  if (rxIndex == 1) rxFrame[0] = SYNC_BYTE;
}

void consumeByte(uint8_t value) {
  if (rxIndex == 0) {
    if (value == SYNC_BYTE) {
      rxFrame[rxIndex++] = value;
    }
    return;
  }
  if (rxIndex == 1) {
    if (value == SYNC_BYTE) {
      rxFrame[rxIndex++] = value;
    } else {
      resetParser(value);
    }
    return;
  }

  if (rxIndex >= sizeof(rxFrame)) {
    resetParser(value);
    return;
  }
  rxFrame[rxIndex++] = value;

  if (rxIndex == 7) {
    const uint8_t payloadLength = rxFrame[6];
    if (payloadLength > MAX_PAYLOAD_LENGTH) {
      // Header is complete; NACK an oversized frame if it is addressed here.
      if (rxFrame[3] == NODE_ID && rxFrame[4] == MSG_REQUEST) {
        rememberSequence(rxFrame[5]);
        sendReply(rxFrame[5], NACK_STATUS, NACK_BAD_LENGTH);
      }
      resetParser(value);
      return;
    }
    expectedFrameLength = 9 + payloadLength;
  }
  if (expectedFrameLength != 0 && rxIndex == expectedFrameLength) {
    handleFrame(expectedFrameLength);
    resetParser(0);
  }
}

void processIncomingUART() {
  // Bound per-loop parsing so a continuous or noisy bus cannot monopolize the
  // CPU and starve the motor sample/timeout checks.
  for (uint8_t consumed = 0; consumed < 32 && Serial.available() > 0; ++consumed) {
    consumeByte((uint8_t)Serial.read());
  }
}

void checkCommandTimeout(unsigned long now) {
  if (!commandTimedOut && now - lastValidCommandMs >= COMMAND_TIMEOUT_MS) {
    setTargetRPM(0, 0.0f);
    setTargetRPM(1, 0.0f);
    commandTimedOut = true;
  }
}

void setup() {
  pinMode(LEFT_EN, OUTPUT);
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_ENC, INPUT_PULLUP);
  pinMode(RIGHT_ENC, INPUT_PULLUP);
  if (RS485_DIRECTION_PIN != 0xFF) {
    pinMode(RS485_DIRECTION_PIN, OUTPUT);
    digitalWrite(RS485_DIRECTION_PIN, !RS485_TX_ACTIVE_LEVEL);
  }

  stopAllMotors();
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC), rightEncoderISR, RISING);
  Serial.begin(115200);

  previousSampleMs = millis();
  lastValidCommandMs = previousSampleMs;
  leftMotor.lastPulseMs = previousSampleMs;
  rightMotor.lastPulseMs = previousSampleMs;
  setTargetRPM(0, START_LEFT_RPM);
  setTargetRPM(1, START_RIGHT_RPM);
}

void loop() {
  processIncomingUART();
  const unsigned long now = millis();
  checkCommandTimeout(now);

  const unsigned long elapsedMs = now - previousSampleMs;
  if (elapsedMs >= SAMPLE_PERIOD_MS) {
    previousSampleMs = now;
    const float dtSeconds = elapsedMs / 1000.0f;

    if (fault == FAULT_NONE) {
      updateMotor(leftMotor, leftEncoderCount, now, dtSeconds,
                  FAULT_LEFT_ENCODER_STALL);
      if (fault == FAULT_NONE) {
        updateMotor(rightMotor, rightEncoderCount, now, dtSeconds,
                    FAULT_RIGHT_ENCODER_STALL);
      } else {
        takeEncoderCount(rightEncoderCount);
      }
    } else {
      takeEncoderCount(leftEncoderCount);
      takeEncoderCount(rightEncoderCount);
      stopAllMotors();
    }
  }
}
