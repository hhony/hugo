#pragma once

#include "pinmap.h"
#include "motor_control.h"
#include "IRremote.h"

#define IR_CMD_FWD        0x00FF18E7  // IR controller ( ▲ ) button
#define IR_CMD_BACK       0x00FF4AB5  // IR controller ( ▼ ) button
#define IR_CMD_RIGHT      0x00FF5AA5  // IR controller ( ▶ ) button
#define IR_CMD_LEFT       0x00FF10EF  // IR controller ( ◀ ) button
#define IR_CMD_STOP_PID   0x00FF38C7  // IR controller ( OK ) button
#define IR_CMD_START_PID  0x00FFB04F  // IR controller ( # ) button
#define IR_CMD_SERVO      0x00FF6897  // IR controller ( * ) button
#define IR_CMD_BEEP       0x00FF9867  // IR controller ( 0 ) button

enum control_signal_t {
  MOVE_FWD,   // forward movement
  MOVE_LEFT,  // left turn
  MOVE_RIGHT, // right turn
  MOVE_BACK,  // backward movement
  MOVE_SERVO, // move servo
  MOVE_STOP_PID,  // kill pid
  TEST_START_PID, // set pid to running
  UNDEFINED
};

control_signal_t _ctrl_sig = UNDEFINED;

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
    if (IRresults.value == IR_CMD_FWD) {
      _ctrl_sig = MOVE_FWD;
    } else if (IRresults.value == IR_CMD_RIGHT) {
      _ctrl_sig = MOVE_RIGHT;
    } else if (IRresults.value == IR_CMD_LEFT) {
      _ctrl_sig = MOVE_LEFT;
    } else if (IRresults.value == IR_CMD_BACK) {
      _ctrl_sig = MOVE_BACK;
    } else if (IRresults.value == IR_CMD_SERVO) {
      _ctrl_sig = MOVE_SERVO;
    } else if (IRresults.value == IR_CMD_START_PID) {
      _ctrl_sig = TEST_START_PID;
    } else if (IRresults.value == IR_CMD_STOP_PID) {
      _ctrl_sig = MOVE_STOP_PID;
    }
    IRresults.value = 0;
    IR.resume();
  }
}
