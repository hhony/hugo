#include "pid_controller.hpp"
#include "sensors_ultrasonic.hpp"

enum robot_state_t {
  ROBOT_STATE_IDLE,
  ROBOT_STATE_STOP,
  ROBOT_STATE_SEARCH,
  ROBOT_STATE_MANUAL_CONTROL
};

enum robot_action_t {
  ROBOT_ACTION_START_SEARCH,
  ROBOT_ACTION_STOP,
  ROBOT_ACTION_MOVE_FORWARD,
  ROBOT_ACTION_MOVE_BACKWARD,
  ROBOT_ACTION_MOVE_RIGHT_TURN
};


robot_state_t robot_state = ROBOT_STATE_IDLE;
robot_state_t next_state = ROBOT_STATE_IDLE;

robot_action_t robot_action = ROBOT_ACTION_STOP;
robot_action_t next_action = ROBOT_ACTION_STOP;


static int left_gain = 0;
static int right_gain = 0;


void robot_stop() {
  left_gain = 0;
  right_gain = 0;
  run_PID_controller(left_gain, right_gain);
}

void process_robot_state() {
  if (robot_state == ROBOT_STATE_IDLE) {
    robot_stop();
    left_gain = 16;
    right_gain = 16;
    next_action = ROBOT_ACTION_STOP;
    next_state = ROBOT_STATE_SEARCH;
  } else if (robot_state == ROBOT_STATE_STOP) {
    if (left_gain || right_gain) {
      robot_stop();
    }
    next_action = ROBOT_ACTION_STOP;
  } else if (robot_state == ROBOT_STATE_SEARCH) {
    if (robot_action == ROBOT_ACTION_STOP) {
      next_action = ROBOT_ACTION_MOVE_FORWARD;
    } else if (robot_action == ROBOT_ACTION_MOVE_FORWARD) {
      if (left_gain == 0 || right_gain == 0) {
        left_gain = 16;
        right_gain = 16;
      }
      if (measure_mm > 600) {
        left_gain = 24;
        right_gain = 24;
      } else if (measure_mm > 400 && measure_mm < 600) {
        left_gain = 12;
        right_gain = 12;
      } else if (measure_mm > 200 && measure_mm < 400) {
        left_gain = 8;
        right_gain = 8;
      } else if (measure_mm < 200) {
        robot_stop();
        next_action = ROBOT_ACTION_MOVE_RIGHT_TURN;
        return;
      }
    } else if (robot_action == ROBOT_ACTION_MOVE_RIGHT_TURN) {
      if (measure_mm > 300) {
        robot_stop();
        next_action = ROBOT_ACTION_MOVE_FORWARD;
      } else {
        left_gain = 16;
        right_gain = -8;
      }
    }
    next_state == ROBOT_STATE_SEARCH;
  }
//  Serial.print("robot_state: ");
//  Serial.print(robot_state);
//  Serial.print(" robot_action: ");
//  Serial.println(robot_action);
//  Serial.print("gains: ");
//  Serial.print(left_gain);
//  Serial.print(", ");
//  Serial.print(right_gain);
//  Serial.print(" output: ");
//  Serial.print(output_left);
//  Serial.print(", ");
//  Serial.print(output_right);
//  Serial.print(" mm: ");
//  Serial.println(measure_mm);
  robot_action = next_action;
  robot_state = next_state;
}