#pragma once

#include "pinmap.h"
#include "PID_v1.h"
#include "motor_control.h"

enum robot_direction_t {
  ROBOT_FORWARD,
  ROBOT_BACKWARD,
  ROBOT_LEFT,
  ROBOT_RIGHT,
  ROBOT_STOP
};

robot_direction_t robot_direction = ROBOT_STOP;

enum motor_direction_t {
  DIR_FORWARD,
  DIR_BACKWARD
};

double kp = 5, ki = 1, kd = 0.01;
double input_left = 0, output_left = 0, setpoint_left = 0;
double input_right = 0, output_right = 0, setpoint_right = 0;

long temp_left = 0, temp_right = 0;
int scale = 500;

volatile long encoder_left_tick = 0;
volatile long encoder_right_tick = 0;

PID pid_left(&input_left, &output_left, &setpoint_left, kp, ki, kd, P_ON_E, DIRECT);
PID pid_right(&input_right, &output_right, &setpoint_right, kp, ki, kd, P_ON_E, DIRECT);

int enc_left_ref = 0, enc_left_int = 0;
int enc_right_ref = 0, enc_right_int = 0;

motor_direction_t _left_direction = DIR_FORWARD;
motor_direction_t _right_direction = DIR_FORWARD;

int ticks_to_mm(int tick) {
  return (int)(tick * 1.175);
}
#define DEFAULT_SPEED 150

void encoder_left() {
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

void encoder_right() {
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

void setup_PID() {
  attachInterrupt(digitalPinToInterrupt(HALL_LEFT_INT), encoder_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_RIGHT_INT), encoder_right, CHANGE);
  set_motor_speed(DEFAULT_SPEED, DEFAULT_SPEED);
  motor_ctrl_stop();
  pid_left.SetMode(AUTOMATIC);
  pid_left.SetSampleTime(1);
  pid_left.SetOutputLimits(-255, 255);
  pid_right.SetMode(AUTOMATIC);
  pid_right.SetSampleTime(1);
  pid_right.SetOutputLimits(-255, 255);
  encoder_left_tick = 0;
  encoder_right_tick = 0;
  temp_left = 0;
  temp_right = 0;
}

void left_PID_controller(int gain) {
  temp_left += gain;
  setpoint_left = temp_left / scale;
  input_left = encoder_left_tick;
  pid_left.Compute();
}

void right_PID_controller(int gain) {
  temp_right += gain;
  setpoint_right = temp_right / scale;
  input_right = encoder_right_tick;
  pid_right.Compute();
}

void run_PID_controller(int left_gain, int right_gain) {
  if (left_gain == 0 && right_gain == 0) {
    temp_left = 0;
    encoder_left_tick  = 0;
    temp_right = 0;
    encoder_right_tick = 0;
  }
  left_PID_controller(left_gain);
  right_PID_controller(right_gain);
  if (left_gain == 0 && right_gain == 0){
    motor_ctrl_stop();
    set_motor_speed(DEFAULT_SPEED, DEFAULT_SPEED);
    robot_direction = ROBOT_STOP;
  } else {
    set_motor_speed((uint8_t) abs(output_left), (uint8_t) abs(output_right));
    if (output_left < 0 && output_right < 0) {
      motor_ctrl_back(0);
      robot_direction = ROBOT_BACKWARD;
    } else if (output_left > 0 && output_right > 0) {
      motor_ctrl_forward(0);
      robot_direction = ROBOT_FORWARD;
    } else if (output_left > 0 && output_right < 0) {
      motor_ctrl_right(0);
      robot_direction = ROBOT_RIGHT;
    } else if (output_left < 0 && output_right > 0) {
      motor_ctrl_left(0);
      robot_direction = ROBOT_LEFT;
    } else {
      motor_ctrl_stop();
      robot_direction = ROBOT_STOP;
    }
  }
}
