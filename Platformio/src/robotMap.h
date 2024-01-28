#pragma once

// BTS7960 motor driver constants
#define RIGHT_R_PWM 3
#define RIGHT_L_PWM 4

#define LEFT_R_PWM 5
#define LEFT_L_PWM 6

#define LOCALIZER_L_PWM_PIN 8
#define LOCALIZER_R_PWM_PIN 9

// MCP2515 Pinout
#define MCP_INT 2 // Brown Wire
#define MCP_CS 53 // Blue Wire
#define MCP_SI 51 // Yellow Wire
#define MCP_SO 50 // Green Wire
#define MCP_SCK 52 // Orange Wire

#define DRIVETRAIN_INTERVAL 10
#define INTERVAL 5000

// Velocity loop PID parameters
// Increase Ki based on load, fine-tune Kp
#define BASE_CURRENT 10
#define SPEED_KP 2500
#define SPEED_KI 4
#define SPEED_KD 0
#define SPEED_SUMCAP 5000

#define POS_KP 0.05

#define LOCALIZER_ENCODER_ID 0
#define LOCALIZER_ENCODER_START_ANGLE -2.23

#define LEFT_WHEELPOD_ENCODER_ID 1
#define LEFT_WHEELPOD_ENCODER_START_ANGLE -0.46319

#define RIGHT_WHEELPOD_ENCODER_ID 2
#define RIGHT_WHEELPOD_ENCODER_START_ANGLE 2.47

#define WHEELPOD_ANGLE_TOLERANCE 0.05

#define FRONT_LEFT_DRIVE_ENCODER_ID 3
#define BACK_LEFT_DRIVE_ENCODER_ID 4
#define BACK_RIGHT_DRIVE_ENCODER_ID 5
#define FRONT_RIGHT_DRIVE_ENCODER_ID 6

#define MULTIPLEXER_0_ID 0 
#define MULTIPLEXER_1_ID 1

#define MAX_MOTOR_CURRENT 10000

#define LEFT_TURN_MOTOR_KP 20
#define RIGHT_TURN_MOTOR_KP 20
#define LOCALIZER_MOTOR_KP 0.01

#define BYTES_2_RAD 0.001533981

#define HYSTERESIS_LIM 0.79
#define HYSTERESIS_BUFFER 0.05