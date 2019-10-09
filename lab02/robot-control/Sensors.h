/*
  File for Sensor APIs
*/

#ifndef Sensors_h
#define Sensors_h

/**
   Enum representing the possible results the sensor data could indicate
*/
enum class SensorResult
{
  Object,
  Nothing,
  Error,
};

class SonicSensor
{
  public:
    SonicSensor(unsigned int triggerPin, unsigned int echoPin, unsigned int distanceThreshold, unsigned long timeout);
    void init();
    void trigger();
    unsigned long getDistance();
    SensorResult getResult();
  private:
    unsigned int triggerPin, echoPin, distanceThreshold;
    unsigned long lastDistance, timeout;
    SensorResult lastResult;
};

#endif
