#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include <Arduino.h>

// --- TB6612FNG Motor Driver Pins ---
// Standby control pin (Active HIGH)
#define PIN_STBY   23  

// Left Motor (Motor A) Control Pins
#define PIN_AIN1   26  // Direction Input 1
#define PIN_AIN2   25  // Direction Input 2
#define PIN_PWMA   33  // Speed Control (PWM)

// Right Motor (Motor B) Control Pins
#define PIN_BIN1   27  // Direction Input 1
#define PIN_BIN2   14  // Direction Input 2
#define PIN_PWMB   32  // Speed Control (PWM)

// --- N20 Quadrature Encoder Pins ---
// Left Motor Encoder Pins
#define PIN_ENC_A_A 16  // Channel A (Interrupt-driven)
#define PIN_ENC_A_B 17  // Channel B (Direction sensing)

// Right Motor Encoder Pins
#define PIN_ENC_B_A 18  // Channel A (Interrupt-driven)
#define PIN_ENC_B_B 19  // Channel B (Direction sensing)

// --- PWM Settings (LEDC for ESP32) ---
#define PWM_FREQ       20000  // 20 kHz PWM (silent motor operation)
#define PWM_RES        8      // 8-bit resolution (0 - 255)
#define PWM_CH_LEFT    0      // LEDC Channel 0 for Left Motor
#define PWM_CH_RIGHT   1      // LEDC Channel 1 for Right Motor

#endif // PIN_DEFINITIONS_H
