#pragma once

static int left_gain = 0;
static int right_gain = 0;

#include "pid_controller.hpp"

void robot_stop(void);
void robot_move(int left_gain, int right_gain);