#include "pinmap.h"

#define ULTRASONIC_MEASURE_US 15

int ultrasonic_measure() {
  long _distance_mm = 0;
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(ULTRASONIC_MEASURE_US);
  _distance_mm = pulseIn(ULTRASONIC_ECHO, HIGH) * 0.1657; // distance in mm
  digitalWrite(ULTRASONIC_TRIG, LOW);
  return (int)_distance_mm;
}
