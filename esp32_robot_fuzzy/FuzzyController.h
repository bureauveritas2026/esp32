#ifndef FUZZY_CONTROLLER_H
#define FUZZY_CONTROLLER_H

#include <Arduino.h>

class FuzzyController {
private:
    float Ge, Gde, Gu;
    float prev_error;
    float integrated_output;
    bool  useIntegration; // true = PI mode (traction), false = PD mode (steering)

    float trimf(float x, float a, float b, float c);
    float trapmf(float x, float a, float b, float c, float d);

public:
    struct FuzzyState {
        float error, d_error, error_norm, d_error_norm;
        float mu_e[5];  // NB, NS, ZE, PS, PB
        float mu_de[3]; // N, ZE, P
        float delta_u, delta_pwm, final_pwm;
    };

    FuzzyState lastState;

    // useIntegration=true → Fuzzy PI (speed/traction)
    // useIntegration=false → Fuzzy PD (position/steering)
    FuzzyController(float ge, float gde, float gu, bool integrate = true);

    float compute(float target, float current, float dt);
    void  reset();
    void  setGains(float ge, float gde, float gu);
    float getGe() const { return Ge; }
    float getGde() const { return Gde; }
    float getGu() const { return Gu; }
};

#endif
