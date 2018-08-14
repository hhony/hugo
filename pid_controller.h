#pragma once

enum robot_direction_t {
  ROBOT_STOP,
  ROBOT_FORWARD,
  ROBOT_BACKWARD,
  ROBOT_LEFT,
  ROBOT_RIGHT
};

static robot_direction_t robot_direction = ROBOT_STOP;


enum motor_direction_t {
  DIR_FORWARD,
  DIR_BACKWARD
};

static motor_direction_t _left_direction = DIR_FORWARD;
static motor_direction_t _right_direction = DIR_FORWARD;


class PIDController {
 private:
  PIDController();

  double _kp = 5, _ki = 0.055, _kd = 0.02;
  double _input_left  = 0, _output_left  = 0, _setpoint_left  = 0;
  double _input_right = 0, _output_right = 0, _setpoint_right = 0;

  int _left_gain = 0, _right_gain = 0;
  long _temp_left = 0, _temp_right = 0;
  double _scale = 2e-3;

  void controller_left(void);
  void controller_right(void);

 public:
  PIDController(PIDController const &) = delete;
  PIDController operator= (PIDController const &) = delete;

  static PIDController &get() {
    static PIDController _context;
    return _context;
  }

  int ticks_to_mm(int tick) { return (int) (tick * 1.175); }

  void setup(void);
  void run(void);
  void stop(void);

  double get_output_left(void)  { return _output_left; }
  double get_output_right(void) { return _output_right; }
  void get_gains(int *left, int *right) { *left = _left_gain; *right = _right_gain; }
  void set_gains(int left, int right) { _left_gain = left; _right_gain = right; }
};