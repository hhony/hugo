#pragma once

#include "sensors_HC_SR04.h"

static HC_SR04 us_sensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
static int measure_mm = 0;

void setup_ultrasonic() {
  us_sensor.begin();
  us_sensor.start();
}

static void ultrasonic_measure() {
  if (us_sensor.is_finished()) {
    measure_mm = us_sensor.get_distance();
    us_sensor.start();
  }
}
