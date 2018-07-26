#pragma once

#include "pinmap.h"

static int measure_mm = 0;

#define ULTRASONIC_MEASURE_US 5

static void ultrasonic_measure() {
  long _distance_mm = 0;
  digitalWrite(ULTRASONIC_TRIG, LOW);
  delayMicroseconds(ULTRASONIC_MEASURE_US);
  digitalWrite(ULTRASONIC_TRIG, HIGH);
  delayMicroseconds(ULTRASONIC_MEASURE_US);
  _distance_mm = pulseIn(ULTRASONIC_ECHO, HIGH) * 0.1657; // distance in mm
  digitalWrite(ULTRASONIC_TRIG, LOW);
  measure_mm = (int)_distance_mm;
}
