#ifndef __HC_SR04_H__
#define __HC_SR04_H__

#include <Arduino.h>

class HC_SR04 {

 private:
  static HC_SR04 *_instance;
  static void _echo_ISR(void);

  int _trigger, _echo;
  volatile unsigned long _begin_us, _ended_us;
  volatile bool _is_finished, _is_triggered;

  float _kalman_err = 0, _kalman_proc_err = 0, _kalman_gain = 0;
  float _kalman_q_process_variance = 2.25e-3;
  float _kalman_r_measure_variance = 3.5e-3;
  float _kalman_xhat = 0;
  unsigned int filter_distance(void);
  unsigned int time_to_mm(void);

 public:
  HC_SR04(int trigger, int echo);

  void begin(void);
  void start(void);
  bool is_finished(void) { return _is_finished; }
  unsigned int get_distance(void);

  static HC_SR04 *context(void) {
    return _instance;
  }

};

#endif
