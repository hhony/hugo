#include "libPinChangeInt.h"
#include "sensors_HC_SR04.h"


#define ULTRASONIC_MEASURE_US 10
HC_SR04 *HC_SR04::_instance(NULL);


HC_SR04::HC_SR04(int trigger, int echo)
    : _trigger(trigger), _echo(echo), _is_finished(false) {
  if (_instance == nullptr)
    _instance = this;
}


void HC_SR04::begin(void) {
  pinMode(_trigger, OUTPUT);
  digitalWrite(_trigger, LOW);
  pinMode(_echo, INPUT);
  PCintPort::attachInterrupt(_echo, &_echo_ISR, CHANGE);
}


void HC_SR04::start(void) {
  _is_finished = false;
  delayMicroseconds(ULTRASONIC_MEASURE_US);
  digitalWrite(_trigger, HIGH);
  delayMicroseconds(ULTRASONIC_MEASURE_US);
  digitalWrite(_trigger, LOW);
  _is_triggered = true;
}


unsigned int HC_SR04::get_distance(void) {
  return (unsigned int) _kalman_xhat;
}


unsigned int HC_SR04::time_to_mm(void) {
  return (unsigned int) ((_ended_us - _begin_us) * 0.1657);
}


unsigned int HC_SR04::filter_distance(void) {
  uint32_t _mm_est = 0;
  if (_ended_us > 0 && _begin_us > 0) {
    _mm_est = time_to_mm();
    _kalman_proc_err = _kalman_err + _kalman_q_process_variance;
    _kalman_gain = _kalman_proc_err / (_kalman_proc_err + _kalman_r_measure_variance);
    _kalman_err  = _kalman_proc_err * (1 - _kalman_gain);
    _kalman_xhat = _kalman_xhat + _kalman_gain * (_mm_est - _kalman_xhat);
  }
}


void HC_SR04::_echo_ISR(void) {
  HC_SR04 *_sensor = HC_SR04::context();
  switch (digitalRead(_sensor->_echo)) {
    case HIGH:
      if (_sensor->_is_triggered) {
        _sensor->_begin_us = micros();
      }
      break;
    case LOW:
      if (_sensor->_is_triggered) {
        _sensor->_ended_us = micros();
        _sensor->filter_distance();
        _sensor->_is_triggered = false;
        _sensor->_is_finished = true;
      }
      break;
  }
}
