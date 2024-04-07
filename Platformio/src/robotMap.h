#pragma once

// Talon pwm motor controller constants
#define RIGHT_TURN_PWM 3

#define LEFT_TURN_PWM 4

#define LOCALIZER_PWM 5

#define DEPOSIT_PWM 6

#define PLUNGE_TALON_PWM 7
#define PLUNGE_MOTOR_EFFORT 30

// MCP2515 Pinout
#define MCP_INT 2 // Brown Wire
#define MCP_CS 53 // Blue Wire
#define MCP_SI 51 // Yellow Wire
#define MCP_SO 50 // Green Wire
#define MCP_SCK 52 // Orange Wire

#define DRIVETRAIN_INTERVAL 10
#define CONVEYOR_INTERVAL 50
#define INTERVAL 5000

// Velocity loop PID parameters
// Increase Ki based on load, fine-tune Kp
#define BASE_CURRENT 10
#define SPEED_KP 6000
#define SPEED_KI 7
#define SPEED_KD 0
#define SPEED_SUMCAP 5000

#define POS_KP 0.05

// Encoder values
#define LOCALIZER_ENCODER_ID 0
#define LOCALIZER_ENCODER_START_ANGLE -2.23

#define LEFT_WHEELPOD_ENCODER_ID 1
#define LEFT_WHEELPOD_ENCODER_START_ANGLE 2.49

#define RIGHT_WHEELPOD_ENCODER_ID 2
#define RIGHT_WHEELPOD_ENCODER_START_ANGLE -2.50

#define WHEELPOD_ANGLE_TOLERANCE 0.05

#define FRONT_LEFT_DRIVE_ENCODER_ID 3
#define BACK_LEFT_DRIVE_ENCODER_ID 4
#define BACK_RIGHT_DRIVE_ENCODER_ID 5
#define FRONT_RIGHT_DRIVE_ENCODER_ID 6

#define DEPOSIT_ENCODER_ID 0
#define DEPOSIT_OPEN_ANGLE 1.89
#define DEPOSIT_CLOSED_ANGLE 0.53

#define CONVEYOR_ENCODER_ID 1
#define CONVEYOR_MULTIPLEXER_ID 1

#define MULTIPLEXER_0_ID 0 
#define MULTIPLEXER_1_ID 1

#define MAX_MOTOR_CURRENT 12000

#define LEFT_TURN_MOTOR_KP 20
#define RIGHT_TURN_MOTOR_KP 20
#define LOCALIZER_MOTOR_KP 0.008
#define LOCALIZER_MOTOR_KI 0.0005
#define MAX_LOCALIZER_ERRORS 1000
#define DEPOSIT_MOTOR_KP 25
#define DEPOSIT_MOTOR_KI 0
#define MAX_DEPOSIT_ERRORS 1000

#define DEPOSIT_ANGLE_THRESHOLD 0.1

#define BYTES_2_RAD 0.001533981

#define HYSTERESIS_LIM 0.79
#define HYSTERESIS_BUFFER 0.05

#define ROBOT_WIDTH_M .5334
#define ROBOT_LENGTH_M .5080

#define ODOM_FREQUENCY 5

#define M_PER_TICK 0.00015585
#define ODOM_CORRECTION_FACTOR 1

#define ANGLE_TO_WHEEL_0 0.809784

#define DISABLED 0
#define DRIVE_STRAIGHT 1
#define POINT_TURN 2
#define ICC_TURN 3

#define ZEB_SPEED 10.24

#define PLUNGE_TOP 43
#define PLUNGE_BOT 42