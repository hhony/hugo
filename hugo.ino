#include "ir_control.h"
#include "pid_controller.h"
#include "servo_control.h"

int left_gain = 0;
int right_gain = 0;

void setup() {
  setup_pinmodes();
  setup_PID();
  setup_ir_control();
  setup_servo();
  Serial.begin(115200);
}


//
// apply IR remote control
//
void apply_IR_commands() {
  switch (_ctrl_sig) {
    case MOVE_FWD:
      left_gain = 128;
      right_gain = 128;
      break;
    case MOVE_LEFT:
      left_gain = -64;
      right_gain = 128;
      break;
    case MOVE_RIGHT:
      left_gain = 128;
      right_gain = -64;
      break;
    case MOVE_BACK:
      left_gain = -128;
      right_gain = -128;
      break;
    case MOVE_STOP_PID:
      left_gain = 0;
      right_gain = 0;
      run_PID_controller(left_gain, right_gain);
      break;
    case MOVE_SERVO:
      move_ultrasonic_servo(3);
      break;
    case TEST_START_PID:
      left_gain = 100;
      right_gain = 100;
      break;
  }
  _ctrl_sig = UNDEFINED;
}


void loop() {
  update_IR_status();
  apply_IR_commands();
//  Serial.println(ultrasonic_measure());
  if (left_gain || right_gain) {
    run_PID_controller(left_gain, right_gain);
  }
}
