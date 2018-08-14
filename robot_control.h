#pragma once

#include "pid_controller.h"

static void robot_stop(void) {
  PIDController::get().stop();
}

static void robot_move() {
  PIDController::get().run();
}

static void robot_set_gains(int left, int right) {
  PIDController::get().set_gains(left, right);
}

static void robot_get_gains(int *left, int *right) {
  PIDController::get().get_gains(left, right);
}
