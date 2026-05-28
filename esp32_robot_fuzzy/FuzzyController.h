#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

#include <Arduino.h>

class FuzzyController {
private:
    // Scaling gains
    float Ge;   // Error gain (scales error to [-1, 1])
    float Gde;  // Change of error gain (scales d_error to [-1, 1])
    float Gu;   // Output scaling gain (scales normalized output to delta PWM)

    float prev_error;
    float integrated_output; // Integrated control signal (PWM value between -255 and 255)

    // Membership functions helper methods
    float trimf(float x, float a, float b, float c);
    float trapmf(float x, float a, float b, float c, float d);

public:
    // Structure to pass internal state of the fuzzy engine for visualization
    struct FuzzyState {
        float error;
        float d_error;
        float error_norm;
        float d_error_norm;
        float mu_e[5];   // Membership values for error {NB, NS, ZE, PS, PB}
        float mu_de[3];  // Membership values for d_error {N, ZE, P}
        float delta_u;   // Normalized delta output [-1, 1]
        float delta_pwm; // Scaled delta output
        float final_pwm; // Final output PWM after integration and saturation [-255, 255]
    };

    FuzzyState lastState;

    FuzzyController(float ge, float gde, float gu);

    // Compute the control action. Returns a value between -255 and 255.
    float compute(float target, float current, float dt);

    // Reset the integrated control signal and previous error
    void reset();

    // Setters for dynamic tuning via MQTT
    void setGains(float ge, float gde, float gu);
    
    // Getters for tuning parameters
    float getGe() const { return Ge; }
    float getGde() const { return Gde; }
    float getGu() const { return Gu; }
};

#endif // FUZZY_CONTROLLER_H
