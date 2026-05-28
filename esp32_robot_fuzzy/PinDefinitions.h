#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include <Arduino.h>

// --- TB6612FNG Motor Driver Pins ---
// Standby control pin (Active HIGH)
#define PIN_STBY   23  

// Traction Motor Control Pins (Rear)
#define PIN_TRAC_IN1   26  // Direction Input 1
#define PIN_TRAC_IN2   25  // Direction Input 2
#define PIN_PWM_TRAC   33  // Speed Control (PWM)

// Steering Motor Control Pins (Front)
#define PIN_STEER_IN1   27  // Direction Input 1
#define PIN_STEER_IN2   14  // Direction Input 2
#define PIN_PWM_STEER   32  // Speed Control (PWM)

// --- N20 Quadrature Encoder Pins ---
// Traction Motor Encoder Pins
#define PIN_ENC_TRAC_A 16  // Channel A (Interrupt-driven)
#define PIN_ENC_TRAC_B 17  // Channel B (Direction sensing)

// Steering Motor Encoder Pins
#define PIN_ENC_STEER_A 18  // Channel A (Interrupt-driven)
#define PIN_ENC_STEER_B 19  // Channel B (Direction sensing)

// --- PWM Settings (LEDC for ESP32) ---
#define PWM_FREQ       20000  // 20 kHz PWM (silent motor operation)
#define PWM_RES        8      // 8-bit resolution (0 - 255)
#define PWM_CH_TRAC    0      // LEDC Channel 0 for Traction Motor
#define PWM_CH_STEER   1      // LEDC Channel 1 for Steering Motor

#endif // PIN_DEFINITIONS_H
