#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include <Arduino.h>

// --- TB6612FNG Motor Driver Pins ---
// (STBY pin is not controlled by ESP32, connected to VCC physically)

// Traction Motor Control Pins (Rear - Motor A)
#define PIN_TRAC_IN1   27  // AIN1
#define PIN_TRAC_IN2   26  // AIN2
#define PIN_PWM_TRAC   14  // PWMA

// Steering Motor Control Pins (Front - Motor B)
#define PIN_STEER_IN1   33  // BIN1
#define PIN_STEER_IN2   32  // BIN2
#define PIN_PWM_STEER   25  // PWMB

// --- N20 Quadrature Encoder Pins ---
// Traction Motor Encoder Pins (Encoder 1)
#define PIN_ENC_TRAC_A  34  // Encoder 1 A (Input-only, external pull-up required)
#define PIN_ENC_TRAC_B  35  // Encoder 1 B (Input-only, external pull-up required)

// Steering Motor Encoder Pins (Encoder 2)
#define PIN_ENC_STEER_A 18  // Encoder 2 A (Internal pull-up supported)
#define PIN_ENC_STEER_B 19  // Encoder 2 B (Internal pull-up supported)

// --- PWM Settings (LEDC for ESP32) ---
#define PWM_FREQ       20000  // 20 kHz PWM (silent motor operation)
#define PWM_RES        8      // 8-bit resolution (0 - 255)
#define PWM_CH_TRAC    0      // LEDC Channel 0 for Traction Motor
#define PWM_CH_STEER   1      // LEDC Channel 1 for Steering Motor

#endif // PIN_DEFINITIONS_H
