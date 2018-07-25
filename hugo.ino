#include "ir_control.h"
#include "pid_controller.hpp"
#include "servo_control.hpp"

int left_gain = 0;
int right_gain = 0;

void setup() {
  setup_pinmodes();
  setup_PID();
  setup_ir_control();
  setup_servo();
  //Serial.begin(115200);
}


void robot_stop() {
  left_gain = 0;
  right_gain = 0;
  run_PID_controller(left_gain, right_gain);
}

//
// apply IR remote control
//
void apply_IR_commands() {
  switch (_ctrl_sig) {
    case MOVE_FWD:
      left_gain = 24;
      right_gain = 24;
      break;
    case MOVE_LEFT:
      left_gain = -8;
      right_gain = 16;
      break;
    case MOVE_RIGHT:
      left_gain = 16;
      right_gain = -8;
      break;
    case MOVE_BACK:
      left_gain = -24;
      right_gain = -24;
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
        case ROBOT_BACKWARD:
          left_gain += 24;
          right_gain += 24;
          break;
        case ROBOT_LEFT:
          left_gain -=8;
          right_gain += 16;
          break;
        case ROBOT_RIGHT:
          left_gain +=16;
          right_gain -= 8;
          break;
        case ROBOT_STOP:
          robot_stop();
          break;
      }
      left_gain = constrain(left_gain, -255, 255);
      right_gain = constrain(right_gain, -255, 255);
      break;
    case MOVE_DECREASE_SPEED:
      switch (robot_direction) {
        case ROBOT_FORWARD:
        case ROBOT_BACKWARD:
          left_gain -= 24;
          right_gain -= 24;
          break;
        case ROBOT_LEFT:
          left_gain +=8;
          right_gain -= 16;
          break;
        case ROBOT_RIGHT:
          left_gain -=16;
          right_gain += 8;
          break;
        case ROBOT_STOP:
          robot_stop();
          break;
      }
      left_gain = constrain(left_gain, -255, 255);
      right_gain = constrain(right_gain, -255, 255);
      break;
  }
  _ctrl_sig = IR_UNDEFINED;
}


void loop() {
  update_IR_status();
  apply_IR_commands();
  if (left_gain || right_gain) {
    run_PID_controller(left_gain, right_gain);
  }
}
