#pragma once

// BTS7960 motor driver constants
#define IS 6
#define R_PWM 3
#define EN 4
#define L_PWM 5

// MCP2515 Pinout
#define MCP_INT 2 // Brown Wire
#define MCP_CS 53 // Blue Wire
#define MCP_SI 51 // Yellow Wire
#define MCP_SO 50 // Green Wire
#define MCP_SCK 52 // Orange Wire

#define INTERVAL 5000

// Velocity loop PID parameters
// Increase Ki based on load, fine-tune Kp
#define SPEED_KP 10
#define SPEED_KI 8
#define SPEED_KD 0.4
#define SPEED_SUMCAP 35

#define POS_KP 0.05