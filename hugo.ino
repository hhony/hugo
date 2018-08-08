#include "ir_control.h"
#include "robot_state.hpp"
#include "servo_control.hpp"


void setup() {
  setup_pinmodes();
  setup_PID();
  setup_ir_control();
  setup_servo();
  setup_ultrasonic();
  //Serial.begin(115200);
}

#define LINEAR_SPEED  18
#define ARC_SPEED     16
#define PIVOT_SPEED   8

//
// apply IR remote control
//
void apply_IR_commands() {
  switch (_ctrl_sig) {
    case MOVE_FWD:
      left_gain = LINEAR_SPEED;
      right_gain = LINEAR_SPEED;
      break;
    case MOVE_LEFT:
      left_gain = -PIVOT_SPEED;
      right_gain = ARC_SPEED;
      break;
    case MOVE_RIGHT:
      left_gain = ARC_SPEED;
      right_gain = -PIVOT_SPEED;
      break;
    case MOVE_BACK:
      left_gain = -LINEAR_SPEED;
      right_gain = -LINEAR_SPEED;
      break;
    case MOVE_STOP_PID:
      robot_stop();
      break;
    case MOVE_SERVO:
      move_ultrasonic_servo(3);
      break;
    case MOVE_INCREASE_SPEED:
      switch (robot_direction) {
        case ROBOT_FORWARD:
          left_gain += LINEAR_SPEED;
          right_gain += LINEAR_SPEED;
          left_gain = constrain(left_gain, 1, 255);
          right_gain = constrain(right_gain, 1, 255);
          break;
        case ROBOT_BACKWARD:
          left_gain -= LINEAR_SPEED;
          right_gain -= LINEAR_SPEED;
          left_gain = constrain(left_gain, -255, -1);
          right_gain = constrain(right_gain, -255, -1);
          break;
        case ROBOT_LEFT:
          left_gain -= PIVOT_SPEED;
          right_gain += ARC_SPEED;
          break;
        case ROBOT_RIGHT:
          left_gain += ARC_SPEED;
          right_gain -= PIVOT_SPEED;
          break;
        case ROBOT_STOP:
          robot_stop();
          break;
      }
      break;
    case MOVE_DECREASE_SPEED:
      switch (robot_direction) {
        case ROBOT_FORWARD:
          left_gain -= LINEAR_SPEED;
          right_gain -= LINEAR_SPEED;
          left_gain = constrain(left_gain, 1, 255);
          right_gain = constrain(right_gain, 1, 255);
          break;
        case ROBOT_BACKWARD:
          left_gain += LINEAR_SPEED;
          right_gain += LINEAR_SPEED;
          left_gain = constrain(left_gain, -255, -1);
          right_gain = constrain(right_gain, -255, -1);
          break;
        case ROBOT_LEFT:
          left_gain += PIVOT_SPEED;
          right_gain -= ARC_SPEED;
          break;
        case ROBOT_RIGHT:
          left_gain -= ARC_SPEED;
          right_gain += PIVOT_SPEED;
          break;
        case ROBOT_STOP:
          robot_stop();
          break;
      }
      break;
  }
  _ctrl_sig = IR_UNDEFINED;
}


void loop() {
  ultrasonic_measure();
  update_IR_status();
  apply_IR_commands();
  if (left_gain || right_gain) {
    run_PID_controller(left_gain, right_gain);
  }
}
