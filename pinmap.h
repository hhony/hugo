#pragma once

// L298N Dual H-Bridge Motor Controller Pins
#define IN1     8   // K1, K2 motor direction
#define IN2     9   // K1, K2 motor direction
#define IN3    10   // K3, K4 motor direction
#define IN4    12   // K3, K4 motor direction
#define ENA     5   // Needs to be a PWM pin to be able to control motor speed ENA
#define ENB     6   // Needs to be a PWM pin to be able to control motor speed ENB
#define IRPIN  13   // IR receiver Signal pin
#define HALL_LEFT_INT   2   // left hall effect interrupt
#define HALL_LEFT_REF   4   // left hall effect reference
#define HALL_RIGHT_INT  3   // right hall effect interrupt
#define HALL_RIGHT_REF  7   // right hall effect reference
#define ULTRASONIC_SERVO  11  // Ultrasonic servo [SG90]
#define ULTRASONIC_ECHO   A3  // Ultrasonic Echo pin
#define ULTRASONIC_TRIG   A2  // Ultrasonic Trigger pin


static void setup_pinmodes() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(HALL_LEFT_INT, INPUT);
  pinMode(HALL_LEFT_REF, INPUT);
  pinMode(HALL_RIGHT_INT, INPUT);
  pinMode(HALL_RIGHT_REF, INPUT);
}