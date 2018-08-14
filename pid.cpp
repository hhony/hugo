#include <Arduino.h>
#include "pid.h"

PID::PID(double *input, double *output, double *setpoint, double kp, double ki, double kd) {
  _output = output;
  _input = input;
  _setpoint = setpoint;
  _freq_ms = 100;
  _kp = kp, _ki = ki, _kd = kd;
  _last_time = millis() - _freq_ms;
}

void PID::set_limits(double lower, double upper) {
  if (lower >= upper)
    return;
  _lower = lower;
  _upper = upper;
}

void PID::set_tuning(double kp, double ki, double kd) {
  double _seconds = _freq_ms / 1000;
  if (kp < 0 || ki < 0 || kd < 0)
    return;
  _kp = kp;
  _ki = ki * _seconds;
  _kd = kd / _seconds;
}

void PID::set_sample_frequency(int milliseconds) {
  if (milliseconds > 0) {
    double rtshift = (double) milliseconds / (double) _freq_ms;
    _ki *= rtshift;
    _kd /= rtshift;
    _freq_ms = (unsigned long) milliseconds;
  }
}

double PID::limit_value(double value) {
  double ret = value;
  if (value > _upper) {
    return _upper;
  } else if (value < _lower) {
    return _lower;
  }
  return ret;
}

void PID::init(void) {
  _output_sum = *_output;
  _last_input = *_input;
  set_sample_frequency(1);
  set_limits(REVERSE_SPEED_LIMIT, FORWARD_SPEED_LIMIT);
  if (_output_sum > _upper)
    _output_sum = _upper;
  else if (_output_sum < _lower)
    _output_sum = _lower;
}

bool PID::run(void) {
  unsigned long new_time = (millis() - _last_time);
  if (new_time >= _freq_ms) {
    double input = *_input;
    double error = *_setpoint - input;
    double diff = (input - _last_input);
    double output = 0;
    _output_sum += (_ki * error);
    _output_sum = limit_value(_output_sum);

    output = (_kp * error) + _output_sum - (_kd * diff);
    *_output = limit_value(output);

    _last_input = input;
    _last_time = new_time;
    return true;
  }
  return false;
}
