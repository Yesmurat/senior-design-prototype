/*

	Wildlife Collision Mitigation System
	Senior Design - Week 1 Prototype

	Ultrasonic object detection with
	LED indicators and buzzer response.

	Revision 0.
	Author: Om Patel

	Revision 1.
	Author: Yesmurat Sagyndyk.
	Summary: the code now controls both an ultrasonic sensor
						and a servo motor. The schematic needs to be changed accordingly.

TODOs:
	1. The schematic needs to be changed accordingly.
	2. An ultrasonic sensor should sit on top of the motor
			to move along with the motor.
	3. Decide on what camera to use in the future instead of an ultrasonic sensor.
	4. Decide if we should switch to something that can support a small LVM...

*/

#include <Servo.h>

// create servo object to control a servo.
Servo myservo;

// variable to store the servo position.
int position = 0;

const int trigPin = 9;
const int echoPin = 10;

const int redLED = 6;
const int greenLED = 7;
const int buzzer = 8;
const int servoPin = 5;

const float detectionDistance = 30.0; // cm

// timing (non-blocking, so the sensor keeps being read while the servo sweeps)
const unsigned long servoStepInterval = 15; // ms between 1 degree steps
const unsigned long sensorInterval = 100;   // ms between distance readings

int servoDirection = 1;
unsigned long lastServoStep = 0;
unsigned long lastSensorRead = 0;

bool objectDetected = false;
int  detectionCount = 0;
int  clearCount = 0;

void setup() {

	Serial.begin(9600);

	// attach the servo to the servo object.
	myservo.attach(servoPin);
	myservo.write(position);

	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);

	pinMode(redLED, OUTPUT);
	pinMode(greenLED, OUTPUT);
	pinMode(buzzer, OUTPUT);

	digitalWrite(trigPin, LOW);
	digitalWrite(redLED, LOW);
	digitalWrite(greenLED, HIGH);
	digitalWrite(buzzer, LOW);

	Serial.println("Wildlife Detection System");
	Serial.println("System initialized");

}

float readDistance() {

	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

	unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);

	// No valid echo received
	if (duration == 0) {
		return -1;
	}

	// Speed of sound is approximately 0.0343 cm/us
	float distance = duration * 0.0343 / 2.0;

	return distance;

}

// Move the servo by 1 degree every servoStepInterval ms, sweeping 0 -> 180 -> 0.
// Return immediately if it is not time for the next step yet.
void stepServo() {

	unsigned long now = millis();
	// mills() returns the number of milliseconds passed since the Arduino board

	if (now - lastServoStep < servoStepInterval) {
		return;
	}

	lastServoStep = now;    
	position += servoDirection;

	if (position >= 180) {
		position = 180;
		servoDirection = -1;
	}
	else if (position <= 0) {
		position = 0;
		servoDirection = 1;
	}

	myservo.write(position);

}

void loop() {

	stepServo();

	// only read the sensor every sensorInterval ms
	if (millis() - lastSensorRead < sensorInterval) {
		return;
	}

	lastSensorRead = millis();

	float distance = readDistance();

	bool validReading = distance >= 2.0 &&
		distance <= 400.0;

	if (validReading) {

		Serial.print("Distance: ");
		Serial.print(distance);
		Serial.print(" cm | ");

		if (distance <= detectionDistance) {

			detectionCount++;
			clearCount = 0;

			if (detectionCount >= 2)
				objectDetected = true;

		}

		else {

			clearCount++;
			detectionCount = 0;

			if (clearCount >= 2)
				objectDetected = false;

		}

	}

	else {

		Serial.print("Distance: No valid echo | ");
		detectionCount = 0;
		clearCount = 0;

	}

	if (objectDetected) {

		digitalWrite(redLED, HIGH);
		digitalWrite(greenLED, LOW);
		digitalWrite(buzzer, HIGH);

		Serial.println("OBJECT DETECTED - WARNING ON");

	}

	else {

		digitalWrite(redLED, LOW);
		digitalWrite(greenLED, HIGH);
		digitalWrite(buzzer, LOW);

		if (validReading)
			Serial.println("CLEAR - WARNING OFF");

		else
			Serial.println("SENSOR READING INVALID");

	}

}
