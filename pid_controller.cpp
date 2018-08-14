#include "pid.h"
#include "pinmap.h"
#include "robot_control.h"

volatile long encoder_left_tick = 0;
volatile long encoder_right_tick = 0;

PID *pid_left;
PID *pid_right;

int enc_left_ref = 0, enc_left_int = 0;
int enc_right_ref = 0, enc_right_int = 0;

// encoder interrupts
static void encoder_left(void);
static void encoder_right(void);
// motor control (directions)
static void motor_ctrl_stop(void);
static void motor_ctrl_forward(int t);
static void motor_ctrl_back(int t);
static void motor_ctrl_right(int t);
static void motor_ctrl_left(int t);
// motor power (speed)
static void set_motor_speed(int lspeed, int rspeed);


//
// PIDController implementation
//
PIDController::PIDController() {
  pid_left  = new PID(&_input_left,  &_output_left,  &_setpoint_left,  _kp, _ki, _kd);
  pid_right = new PID(&_input_right, &_output_right, &_setpoint_right, _kp, _ki, _kd);
}

void PIDController::setup() {
  attachInterrupt(digitalPinToInterrupt(HALL_LEFT_INT),  encoder_left,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT_INT), encoder_right, CHANGE);
  set_motor_speed(DEFAULT_SPEED, DEFAULT_SPEED);
  motor_ctrl_stop();
  pid_left->init();
  pid_right->init();
  encoder_left_tick = 0;
  encoder_right_tick = 0;
  _temp_left = 0;
  _temp_right = 0;
}

void PIDController::controller_left() {
  _temp_left += _left_gain;
  _setpoint_left = _scale * _temp_left;
  _input_left = encoder_left_tick;
  pid_left->run();
}

void PIDController::controller_right() {
  _temp_right += _right_gain;
  _setpoint_right = _scale * _temp_right;
  _input_right = encoder_right_tick;
  pid_right->run();
}

void PIDController::run() {
  if ((_left_gain == 0 && encoder_left_tick != 0) || (_right_gain == 0 && encoder_right_tick != 0)) {
    _temp_left = 0; _temp_right = 0;
    encoder_left_tick = 0; encoder_right_tick = 0;
    motor_ctrl_stop();
    set_motor_speed(0, 0);
  }

  controller_left();
  controller_right();

  if (_left_gain == 0 && _right_gain == 0){
    robot_direction = ROBOT_STOP;
  } else {
    set_motor_speed((uint8_t) abs(_output_left), (uint8_t) abs(_output_right));
    if (_output_left < 0 && _output_right < 0) {
      motor_ctrl_back(0);
      robot_direction = ROBOT_BACKWARD;
    } else if (_output_left > 0 && _output_right > 0) {
      motor_ctrl_forward(0);
      robot_direction = ROBOT_FORWARD;
    } else if (_output_left > 0 && _output_right < 0) {
      motor_ctrl_right(0);
      robot_direction = ROBOT_RIGHT;
    } else if (_output_left < 0 && _output_right > 0) {
      motor_ctrl_left(0);
      robot_direction = ROBOT_LEFT;
    } else {
      motor_ctrl_stop();
      robot_direction = ROBOT_STOP;
    }
  }
}

void PIDController::stop(void) {
  _left_gain = 0; _right_gain = 0;
  run();
}


//
// encoder interrupts
//
static void encoder_left(void) {
  enc_left_int = digitalRead(HALL_LEFT_INT);
  enc_left_ref = digitalRead(HALL_LEFT_REF);
  if (enc_left_int ^ enc_left_ref) {  // XOR when moving backwards
    _left_direction = DIR_BACKWARD;
    encoder_left_tick--;
  } else {
    _left_direction = DIR_FORWARD;
    encoder_left_tick++;
  }
}

static void encoder_right(void) {
  enc_right_int = digitalRead(HALL_RIGHT_INT);
  enc_right_ref = digitalRead(HALL_RIGHT_REF);
  if (enc_right_int ^ enc_right_ref) { // XOR when moving forwards
    _right_direction = DIR_FORWARD;
    encoder_right_tick++;
  } else {
    _right_direction = DIR_BACKWARD;
    encoder_right_tick--;
  }
}


//
// motor control
//
static void motor_ctrl_stop(void) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

static void motor_ctrl_forward(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(t);
}

static void motor_ctrl_back(int t) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(t);
}

static void motor_ctrl_right(int t) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(t);
}

static void motor_ctrl_left(int t) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(t);
}


//
// motor speed
//
static void set_motor_speed(int lspeed, int rspeed) {
  analogWrite(ENA, lspeed);
  analogWrite(ENB, rspeed);
}
