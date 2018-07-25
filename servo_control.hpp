#pragma once

#include "pinmap.h"
#include <Servo.h>

Servo _head;

int angle_offset = 14;

void move_ultrasonic_servo(int _speed, int _limit=180) {
  int i;
  for (i = angle_offset; i < _limit; i++) {
    _head.write(i + angle_offset);
    delay(_speed);
  }
  for (i = _limit; i >= angle_offset; i--) {
    _head.write(i - angle_offset);
    delay(_speed);
  }
  _head.write(90 - angle_offset);
}

void setup_servo() {
  _head.attach(ULTRASONIC_SERVO);
  move_ultrasonic_servo(1, 90);
}
