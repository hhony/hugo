#pragma once

#include <string.h>
#include "sensors_ultrasonic.hpp"

enum robot_state_t {
  ROBOT_STATE_IDLE,
  ROBOT_STATE_STOP,
  ROBOT_STATE_SEARCH,
  ROBOT_STATE_MANUAL_CONTROL
};

enum robot_action_t {
  ROBOT_ACTION_START_SEARCH,
  ROBOT_ACTION_STOP_SEARCH,
  ROBOT_ACTION_STOP,
  ROBOT_ACTION_MOVE_FORWARD,
  ROBOT_ACTION_MOVE_BACKWARD,
  ROBOT_ACTION_MOVE_RIGHT_TURN
};

robot_state_t robot_state = ROBOT_STATE_MANUAL_CONTROL;
robot_state_t next_state = ROBOT_STATE_MANUAL_CONTROL;

robot_action_t robot_action = ROBOT_ACTION_STOP;
robot_action_t next_action = ROBOT_ACTION_STOP;

#define CASE_RETURN_STRING( str )           \
    case ( ( str ) ): return( (char*)(#str) );

char *trace_robot_state(uint32_t code) {
  switch(code) {
    CASE_RETURN_STRING(ROBOT_STATE_IDLE);
    CASE_RETURN_STRING(ROBOT_STATE_STOP);
    CASE_RETURN_STRING(ROBOT_STATE_SEARCH);
    CASE_RETURN_STRING(ROBOT_STATE_MANUAL_CONTROL);
    default:
      return (char*)"UNDEFINED_STATE";
  }
}

char *trace_robot_action(uint32_t code) {
  switch(code) {
    CASE_RETURN_STRING(ROBOT_ACTION_START_SEARCH);
    CASE_RETURN_STRING(ROBOT_ACTION_STOP_SEARCH);
    CASE_RETURN_STRING(ROBOT_ACTION_STOP);
    CASE_RETURN_STRING(ROBOT_ACTION_MOVE_FORWARD);
    CASE_RETURN_STRING(ROBOT_ACTION_MOVE_BACKWARD);
    CASE_RETURN_STRING(ROBOT_ACTION_MOVE_RIGHT_TURN);
    default:
      return (char*)"UNDEFINED_ACTION";
  }
}

void debug_robot_state() {
  static int cnt = 0;
  if (cnt++ > 100) {
    cnt = 0;
    Serial.print("robot_state: ");
    Serial.print(trace_robot_state(robot_state));
    Serial.print(" robot_action: ");
    Serial.println(trace_robot_action(robot_action));
    Serial.print("gains: ");
    Serial.print(left_gain);
    Serial.print(", ");
    Serial.print(right_gain);
    Serial.print(" output: ");
    Serial.print(output_left);
    Serial.print(", ");
    Serial.print(output_right);
    Serial.print(" mm: ");
    Serial.println(measure_mm);
  }
}


void process_robot_state() {
  if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
    if (left_gain || right_gain) {
      run_PID_controller(left_gain, right_gain);
      next_state = ROBOT_STATE_MANUAL_CONTROL;
    }
  } else if (robot_state == ROBOT_STATE_IDLE) {
    if (robot_action == ROBOT_ACTION_START_SEARCH) {
      robot_stop();
      left_gain = 16;
      right_gain = 16;
      next_action = ROBOT_ACTION_STOP;
      next_state = ROBOT_STATE_SEARCH;
    }
  } else if (robot_state == ROBOT_STATE_STOP) {
    if (left_gain || right_gain) {
      robot_stop();
    }
    next_action = ROBOT_ACTION_STOP;
  } else if (robot_state == ROBOT_STATE_SEARCH) {
    if (robot_action == ROBOT_ACTION_STOP_SEARCH) {
      robot_stop();
      next_state = ROBOT_STATE_MANUAL_CONTROL;
      return;
    } else if (robot_action == ROBOT_ACTION_STOP) {
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
    if (left_gain || right_gain) {
      run_PID_controller(left_gain, right_gain);
      next_state = ROBOT_STATE_SEARCH;
    }
  }
  //debug_robot_state();
  robot_action = next_action;
  robot_state = next_state;
}