#include "Sensors.h"
#include "Arduino.h"

SonicSensor::SonicSensor(unsigned int triggerPin, unsigned int echoPin, unsigned int distanceThreshold, unsigned long timeout)
{
  this->triggerPin = triggerPin;
  this->echoPin = echoPin;
  this->distanceThreshold = distanceThreshold;
  this->timeout = timeout;
}

void SonicSensor::init() {
  pinMode(this->triggerPin, OUTPUT);
  pinMode(this->echoPin, INPUT);
  if (Serial) {
    Serial.print("Sonic Sensor initialized using pins "); Serial.print(triggerPin); Serial.print(" and "); Serial.println(echoPin);
  }
}

void SonicSensor::trigger() {
  Serial.print("Triggering sonic sensor on pin "); Serial.println(this->triggerPin);
  digitalWrite(this->triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->triggerPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(this->triggerPin, LOW);
  delayMicroseconds(2);

  Serial.print("Reading pulse on pin "); Serial.println(this->echoPin);
  unsigned long duration = pulseIn(this->echoPin, HIGH, this->timeout);

  if (duration == 0) {
    this->lastDistance = -1;
    this->lastResult = SensorResult::Error;
    if (Serial) {
      Serial.print("Timeout detected on sonic sensor after "); Serial.print(this->timeout); Serial.println(" microseconds");
    }
    return;
  }
  
  unsigned long cm = duration / 29 / 2;
  this->lastDistance = cm;
  
  if (cm >= this->distanceThreshold) {
    this->lastResult = SensorResult::Nothing;
  } else {
    this->lastResult = SensorResult::Object;
  }
}

unsigned long SonicSensor::getDistance() {
  return this->lastDistance;
}

SensorResult SonicSensor::getResult() {
  return this->lastResult;
}

IRSensor::IRSensor(unsigned int outPin) {
  this->outPin = outPin;
}

void IRSensor::init() {
  pinMode(outPin, INPUT);
  if (Serial) {
    Serial.print("Infrared Sensor initialized using pin "); Serial.println(this->outPin);
  }
}

SensorResult IRSensor::getResult() {
  if (digitalRead(this->outPin) == LOW) {
    return SensorResult::Object;
  } else {
    return SensorResult::Nothing;
  }
}
