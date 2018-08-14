#include "sensors_ir_control.hpp"

void setup() {
  setup_pinmodes();
  PIDController::get().setup();
  setup_ir_control();
  setup_servo();
  setup_ultrasonic();
  //Serial.begin(115200);
}


void loop() {
  ultrasonic_measure();
  update_IR_status();
  apply_IR_commands();
  process_robot_state();
}
