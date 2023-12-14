#pragma once

// BTS7960 motor driver constants
#define RIGHT_R_PWM 3
#define RIGHT_L_PWM 4

#define LEFT_R_PWM 5
#define LEFT_L_PWM 6

// MCP2515 Pinout
#define MCP_INT 2 // Brown Wire
#define MCP_CS 53 // Blue Wire
#define MCP_SI 51 // Yellow Wire
#define MCP_SO 50 // Green Wire
#define MCP_SCK 52 // Orange Wire

#define INTERVAL 5000

// Velocity loop PID parameters
// Increase Ki based on load, fine-tune Kp
#define SPEED_KP 5
#define SPEED_KI 5
#define SPEED_KD 0.4
#define SPEED_SUMCAP 10

#define POS_KP 0.05

#define LOCALIZER_ENCODER_ID 0
#define LOCALIZER_START_ANGLE 0.0

#define LEFT_WHEELPOD_ENCODER_ID 1
#define LEFT_WHEELPOD_ENCODER_START_ANGLE -0.521

#define RIGHT_WHEELPOD_ENCODER_ID 2
#define RIGHT_WHEELPOD_ENCODER_START_ANGLE 2.60
#define MAX_MOTOR_CURRENT 10000

#define LEFT_TURN_MOTOR_KP 20
#define RIGHT_TURN_MOTOR_KP 20
