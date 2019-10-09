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
}

void SonicSensor::trigger() {
  digitalWrite(this->triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(this->triggerPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(this->triggerPin, LOW);
  delayMicroseconds(2);

  unsigned long duration = pulseIn(this->echoPin, HIGH, this->timeout);

  if (duration == 0) {
    this->lastDistance = -1;
    this->lastResult = SensorResult::Error;
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
