#pragma once

#include "robot_control.h"
#include "sensors_ultrasonic.hpp"
#include "servo_control.hpp"

static int left_gain = 0, right_gain = 0;


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
  ROBOT_ACTION_MOVE_RIGHT_TURN,
  ROBOT_ACTION_MOVE_LEFT_TURN
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
    Serial.print(PIDController::get().get_output_left());
    Serial.print(", ");
    Serial.print(PIDController::get().get_output_right());
    Serial.print(" mm: ");
    Serial.println(measure_mm);
  }
}


#define MAX_DISTANCE_AWARE  800
#define MAX_DISTANCE_SLOWS  650
#define MAX_DISTANCE_AVOID  215
#define MAX_DISTANCE_CLEAR  420
#define MAX_DISTANCE_TURNS  650

#define ROBOT_SPEED_FAST    24
#define ROBOT_SPEED_HIGH    16
#define ROBOT_SPEED_MID     12
#define ROBOT_SPEED_SLOW    8


int us_adjust_angle = 90;
int us_adjust_count = 0;
int us_adjust_wait = 1000;
int us_adjust_tick = 12;
int us_last_direction = 0;

bool follow_right_clear() {
  bool ret = false;
  if (us_adjust_count++ > us_adjust_wait) {
    us_adjust_count = 0;
    if (us_adjust_angle < 120) {
      us_adjust_angle += us_adjust_tick;
    } else {
      us_adjust_angle = 90;
      ret = true;
    }
    command_ultrasonic_angle(us_adjust_angle);
  }
  return ret;
}

bool follow_left_clear() {
  bool ret = false;
  if (us_adjust_count++ > us_adjust_wait) {
    us_adjust_count = 0;
    if (us_adjust_angle > 60) {
      us_adjust_angle -= us_adjust_tick;
    } else {
      us_adjust_angle = 90;
      ret = true;
    }
    command_ultrasonic_angle(us_adjust_angle);
  }
  return ret;
}

robot_action_t get_turn_direction() {
  static int last_turn = 0;
  if (last_turn++ % 2) {
    return ROBOT_ACTION_MOVE_RIGHT_TURN;
  }
  return ROBOT_ACTION_MOVE_LEFT_TURN;
}


void process_robot_state() {
  if (robot_state == ROBOT_STATE_MANUAL_CONTROL) {
    robot_get_gains(&left_gain, &right_gain);
    if (left_gain || right_gain) {
      robot_move();
    } else {
      robot_stop();
    }
    next_state = ROBOT_STATE_MANUAL_CONTROL;

  } else if (robot_state == ROBOT_STATE_IDLE) {
    if (robot_action == ROBOT_ACTION_START_SEARCH) {
      robot_stop();
      left_gain = ROBOT_SPEED_MID;
      right_gain = ROBOT_SPEED_MID;
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

    } else if (robot_action == ROBOT_ACTION_STOP) {
      next_action = ROBOT_ACTION_MOVE_FORWARD;

    } else if (robot_action == ROBOT_ACTION_MOVE_FORWARD) {
      if (left_gain == 0 || right_gain == 0) {
        left_gain = ROBOT_SPEED_HIGH;
        right_gain = ROBOT_SPEED_HIGH;
      }

      if (measure_mm > MAX_DISTANCE_AWARE) {
        left_gain = ROBOT_SPEED_FAST;
        right_gain = ROBOT_SPEED_FAST;
      } else if (measure_mm > MAX_DISTANCE_SLOWS && measure_mm < MAX_DISTANCE_AWARE) {
        left_gain = ROBOT_SPEED_MID;
        right_gain = ROBOT_SPEED_MID;
      } else if (measure_mm > MAX_DISTANCE_AVOID && measure_mm < MAX_DISTANCE_SLOWS) {
        left_gain = ROBOT_SPEED_SLOW;
        right_gain = ROBOT_SPEED_SLOW;
      } else if (measure_mm < MAX_DISTANCE_AVOID) {
        robot_stop();
        next_action = ROBOT_ACTION_MOVE_BACKWARD;
        return;
      }

    } else if (robot_action == ROBOT_ACTION_MOVE_BACKWARD) {
      if (measure_mm > MAX_DISTANCE_CLEAR) {
        next_action = get_turn_direction();
      }
      left_gain = -ROBOT_SPEED_SLOW;
      right_gain = -ROBOT_SPEED_SLOW;

    } else if (robot_action == ROBOT_ACTION_MOVE_RIGHT_TURN) {
      if (measure_mm > MAX_DISTANCE_TURNS) {
        if (follow_right_clear()) {
          robot_stop();
          next_action = ROBOT_ACTION_MOVE_FORWARD;
        }
      }
      left_gain = ROBOT_SPEED_HIGH;
      right_gain = -ROBOT_SPEED_SLOW;

    } else if (robot_action == ROBOT_ACTION_MOVE_LEFT_TURN){
      if (measure_mm > MAX_DISTANCE_TURNS) {
        if (follow_left_clear()) {
          robot_stop();
          next_action = ROBOT_ACTION_MOVE_FORWARD;
        }
      }
      left_gain = -ROBOT_SPEED_SLOW;
      right_gain = ROBOT_SPEED_HIGH;
    }

    if (left_gain || right_gain) {
      robot_set_gains(left_gain, right_gain);
      robot_move();
      next_state = ROBOT_STATE_SEARCH;
    }
  }
  //debug_robot_state();
  robot_action = next_action;
  robot_state = next_state;
}