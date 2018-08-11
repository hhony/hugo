#ifndef __PID_H__
#define __PID_H__

#define DEFAULT_SPEED         150
#define REVERSE_SPEED_LIMIT  -255
#define FORWARD_SPEED_LIMIT   255

class PID {
 public:
  PID (double *input, double *output, double *setpoint, double kp, double ki, double kd);

  void init(void);
  bool run(void);
  void set_limits(double lower, double upper);
  void set_tuning(double kp, double ki, double kd);
  void set_sample_frequency(int milliseconds);

 private:
  double _kp, _ki, _kd;

  double * _input;
  double * _output;
  double * _setpoint;

  unsigned long _last_time, _freq_ms;
  double _output_sum, _last_input;

  double _lower, _upper;

  double limit_value(double value);
};

#endif // __PID_H__
