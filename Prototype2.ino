/*
  Wildlife Collision Mitigation - Prototype 2
  ELEGOO / Arduino Mega 2560 + HC-SR04 + SG90 servo

  Existing Prototype 1 wiring (UNCHANGED):
  HC-SR04: VCC -> Arduino 5V; GND -> Arduino GND
           TRIG -> D9; ECHO -> D10
  Red LED: D6 -> 220-ohm resistor -> LED anode; cathode -> GND
  Green LED: D7 -> 220-ohm resistor -> LED anode; cathode -> GND
  Active buzzer: + -> D8; - -> GND

  New servo wiring:
  SG90 signal (orange/yellow) -> Arduino D11
  SG90 power (red) -> EXTERNAL regulated 5V +
  SG90 ground (brown/black) -> EXTERNAL 5V ground (-)
  EXTERNAL 5V ground (-) -> Arduino GND (SHARED GROUND)
  DO NOT connect external 5V + to Arduino 5V.

  Aiming/limitations:
  - Step to each angle; STOP & SETTLE before measuring.
  - Take 3 range readings; use 2 matching or 3 with median filtering.
  - Trigger only when a nearby object appears at the SAME angle
    on at least two successive visits to that angle.
  - This detects nearby objects, NOT animals or roadway trajectories.
*/

#include <Servo.h>

// Prototype 1 pins, unchanged
const byte trigPin = 9;
const byte echoPin = 10;
const byte redLED = 6;
const byte greenLED = 7;
const byte buzzer = 8;

// New servo pin
const byte servoPin = 11;
Servo scanServo;

// Editable scanning settings
const int MIN_ANGLE = 30;             // degrees
const int MAX_ANGLE = 150;            // degrees
const int ANGLE_STEP = 15;            // degrees: 9 scan positions
const unsigned long SETTLE_MS = 180;  // wait after EACH servo step
const unsigned long PING_GAP_MS = 65; // time between ultrasonic pings

// Editable detection settings (centimeters)
const float DETECT_CM = 30.0;
const float MIN_TARGET_CM = 8.0;      // reject very close mounting echoes
const float MAX_VALID_CM = 400.0;
const float PAIR_TOLERANCE_CM = 10.0; // if only 2 valid pings, must agree
const byte CONFIRM_VISITS = 2;        // same angle, two successive visits
const byte CLEAR_PASSES = 2;         // whole passes without a confirmation

const byte POSITION_COUNT = (MAX_ANGLE - MIN_ANGLE) / ANGLE_STEP + 1;
byte consecutiveHits[POSITION_COUNT] = {0};
bool confirmedThisPass = false;
bool warningActive = false;
byte clearPassCount = 0;

// Read a single HC-SR04 measurement in cm. -1 = no valid echo.
float readDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 25000UL);
  if (duration == 0) return -1.0;

  // Approximate speed of sound at room temperature, cm/us
  return duration * 0.0343f / 2.0f;
}

// Returns a filtered distance or -1 if the measurement is unreliable.
float readFilteredDistanceCm() {
  float readings[3];
  byte validCount = 0;

  for (byte i = 0; i < 3; ++i) {
    float cm = readDistanceCm();
    if (cm >= MIN_TARGET_CM && cm <= MAX_VALID_CM) {
      readings[validCount++] = cm;
    }
    if (i < 2) delay(PING_GAP_MS);
  }

  if (validCount < 2) return -1.0;

  if (validCount == 2) {
    float difference = readings[0] - readings[1];
    if (difference < 0) difference = -difference;
    if (difference > PAIR_TOLERANCE_CM) return -1.0;
    return (readings[0] + readings[1]) / 2.0f;
  }

  // Sort 3 values so readings[1] is the median.
  if (readings[0] > readings[1]) {
    float t = readings[0]; readings[0] = readings[1]; readings[1] = t;
  }
  if (readings[1] > readings[2]) {
    float t = readings[1]; readings[1] = readings[2]; readings[2] = t;
  }
  if (readings[0] > readings[1]) {
    float t = readings[0]; readings[0] = readings[1]; readings[1] = t;
  }
  return readings[1];
}

void updateOutputs() {
  digitalWrite(redLED, warningActive ? HIGH : LOW);
  digitalWrite(greenLED, warningActive ? LOW : HIGH);
  digitalWrite(buzzer, warningActive ? HIGH : LOW);
}

void scanAngle(int angle) {
  // Move first, then measure only after the servo has settled.
  scanServo.write(angle);
  delay(SETTLE_MS);

  float cm = readFilteredDistanceCm();
  byte index = (angle - MIN_ANGLE) / ANGLE_STEP;
  bool nearObject = (cm >= MIN_TARGET_CM && cm <= DETECT_CM);

  if (nearObject) {
    if (consecutiveHits[index] < CONFIRM_VISITS) {
      ++consecutiveHits[index];
    }
  } else {
    // A reading from a different angle cannot build a hit streak.
    consecutiveHits[index] = 0;
  }

  if (nearObject && consecutiveHits[index] >= CONFIRM_VISITS) {
    confirmedThisPass = true;
    clearPassCount = 0;
    warningActive = true;
  }

  updateOutputs();

  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" deg | Distance: ");
  if (cm < 0) Serial.print("INVALID");
  else {
    Serial.print(cm, 1);
    Serial.print(" cm");
  }
  Serial.print(" | Hits at angle: ");
  Serial.print(consecutiveHits[index]);
  Serial.print(" | Warning: ");
  Serial.println(warningActive ? "ON" : "OFF");
}

// Called after a full direction pass across the detection sector.
void finishPass() {
  if (confirmedThisPass) {
    clearPassCount = 0;
  } else if (warningActive) {
    ++clearPassCount;
    if (clearPassCount >= CLEAR_PASSES) {
      warningActive = false;
      clearPassCount = 0;
    }
  }
  updateOutputs();
  confirmedThisPass = false;
}

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(trigPin, LOW);

  warningActive = false;
  updateOutputs();

  // Move to center first. Mount the horn after centering if necessary.
  scanServo.write(90);
  scanServo.attach(servoPin);
  delay(700);

  Serial.println("Wildlife Collision Mitigation - Prototype 2");
  Serial.println("Servo sweep: 30 to 150 degrees, then back.");
  Serial.println("Detection requires two visits to the same angle.");
}

void loop() {
  Serial.println("--- Forward sweep ---");
  for (int angle = MIN_ANGLE; angle <= MAX_ANGLE; angle += ANGLE_STEP) {
    scanAngle(angle);
  }
  finishPass();

  Serial.println("--- Reverse sweep ---");
  // Do not repeat the 150-degree endpoint immediately on reversal.
  for (int angle = MAX_ANGLE - ANGLE_STEP; angle >= MIN_ANGLE; angle -= ANGLE_STEP) {
    scanAngle(angle);
  }
  finishPass();
}
