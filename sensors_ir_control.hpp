#pragma once

#include "pinmap.h"
#include "robot_control.h"
#include "libIRremote.h"
#include "servo_control.hpp"

#define IR_CMD_UP_ARROW     0x00FF18E7  // IR controller ( ▲ ) button
#define IR_CMD_DOWN_ARROW   0x00FF4AB5  // IR controller ( ▼ ) button
#define IR_CMD_RIGHT_ARROW  0x00FF5AA5  // IR controller ( ▶ ) button
#define IR_CMD_LEFT_ARROW   0x00FF10EF  // IR controller ( ◀ ) button
#define IR_CMD_OK           0x00FF38C7  // IR controller ( OK ) button
#define IR_CMD_HASH         0x00FFB04F  // IR controller ( # ) button
#define IR_CMD_STAR         0x00FF6897  // IR controller ( * ) button
#define IR_CMD_ZERO         0x00FF9867  // IR controller ( 0 ) button
#define IR_CMD_ONE          0x00FFA25D  // IR controller ( 1 ) button
#define IR_CMD_TWO          0x00FF629D  // IR controller ( 2 ) button
#define IR_CMD_THREE        0x00FFE21D  // IR controller ( 3 ) button
#define IR_CMD_FOUR         0x00FF22DD  // IR controller ( 4 ) button
#define IR_CMD_FIVE         0x00FF02FD  // IR controller ( 5 ) button
#define IR_CMD_SIX          0x00FFC23D  // IR controller ( 6 ) button
#define IR_CMD_SEVEN        0x00FFE01F  // IR controller ( 7 ) button
#define IR_CMD_EIGHT        0x00FFA857  // IR controller ( 8 ) button
#define IR_CMD_NINE         0x00FF906F  // IR controller ( 9 ) button


enum control_signal_t {
  MOVE_FWD,   // forward movement
  MOVE_LEFT,  // left turn
  MOVE_RIGHT, // right turn
  MOVE_BACK,  // backward movement
  MOVE_SERVO, // move servo
  MOVE_STOP_PID,  // kill pid
  MOVE_INCREASE_SPEED, // increase pid speed
  MOVE_DECREASE_SPEED, // decrease pid speed
  IR_UNDEFINED
};

control_signal_t _ctrl_sig = IR_UNDEFINED;

// IR Receiver object
IRrecv IR( IRPIN );
decode_results IRresults;

void setup_ir_control() {
  pinMode(IRPIN, INPUT);
  digitalWrite(IRPIN, HIGH);
  IR.enableIRIn();
}

//
// detect IR code
//
void update_IR_status() {
  if (IR.decode(&IRresults)) {
    switch (IRresults.value) {

      case IR_CMD_UP_ARROW:
        _ctrl_sig = MOVE_FWD;
        break;

      case IR_CMD_RIGHT_ARROW:
        _ctrl_sig = MOVE_RIGHT;
        break;

      case IR_CMD_LEFT_ARROW:
        _ctrl_sig = MOVE_LEFT;
        break;

      case IR_CMD_DOWN_ARROW:
        _ctrl_sig = MOVE_BACK;
        break;

      case IR_CMD_HASH:
        _ctrl_sig = MOVE_INCREASE_SPEED;
        break;

      case IR_CMD_STAR:
        _ctrl_sig = MOVE_DECREASE_SPEED;
        break;

      case IR_CMD_OK:
        _ctrl_sig = MOVE_STOP_PID;
        break;

      case IR_CMD_ZERO:
        _ctrl_sig = MOVE_SERVO;
        break;

      case IR_CMD_ONE:
      case IR_CMD_TWO:
      case IR_CMD_THREE:
      case IR_CMD_FOUR:
      case IR_CMD_FIVE:
      case IR_CMD_SIX:
      case IR_CMD_SEVEN:
      case IR_CMD_EIGHT:
      case IR_CMD_NINE:
      default:
        _ctrl_sig = IR_UNDEFINED;
        break;
    }
    IRresults.value = 0;
    IR.resume();
  }
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
