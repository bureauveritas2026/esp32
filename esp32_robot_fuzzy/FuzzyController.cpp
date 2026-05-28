#include "FuzzyController.h"

FuzzyController::FuzzyController(float ge, float gde, float gu, bool integrate) {
    Ge = ge; Gde = gde; Gu = gu;
    useIntegration = integrate;
    prev_error = 0.0f;
    integrated_output = 0.0f;
    memset(&lastState, 0, sizeof(FuzzyState));
}

float FuzzyController::trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

float FuzzyController::trapmf(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x < b) return (x - a) / (b - a);
    return (d - x) / (d - c);
}

float FuzzyController::compute(float target, float current, float dt) {
    float error   = target - current;
    float d_error = error - prev_error;

    float en  = constrain(error   * Ge,  -1.0f, 1.0f);
    float den = constrain(d_error * Gde, -1.0f, 1.0f);

    // Fuzzify error (5 sets)
    float mu_e[5];
    mu_e[0] = trapmf(en, -2.0f,-2.0f,-0.6f,-0.2f); // NB
    mu_e[1] = trimf(en, -0.5f,-0.25f, 0.0f);        // NS
    mu_e[2] = trimf(en, -0.2f, 0.0f,  0.2f);        // ZE
    mu_e[3] = trimf(en,  0.0f, 0.25f, 0.5f);        // PS
    mu_e[4] = trapmf(en,  0.2f, 0.6f, 2.0f, 2.0f); // PB

    // Fuzzify d_error (3 sets)
    float mu_de[3];
    mu_de[0] = trapmf(den,-2.0f,-2.0f,-0.4f, 0.0f); // N
    mu_de[1] = trimf(den, -0.4f, 0.0f, 0.4f);        // ZE
    mu_de[2] = trapmf(den,  0.0f, 0.4f, 2.0f, 2.0f);// P

    // Save to state
    for(int i=0;i<5;i++) lastState.mu_e[i]  = mu_e[i];
    for(int i=0;i<3;i++) lastState.mu_de[i] = mu_de[i];

    // Rule base: singletons [error][d_error]
    const float rs[5][3] = {
        {-1.0f,-1.0f,-0.6f},
        {-0.6f,-0.3f, 0.0f},
        {-0.3f, 0.0f, 0.3f},
        { 0.0f, 0.3f, 0.6f},
        { 0.6f, 1.0f, 1.0f}
    };

    float sum_fs = 0.0f, sum_wo = 0.0f;
    for(int i=0;i<5;i++) for(int j=0;j<3;j++) {
        float fs = min(mu_e[i], mu_de[j]);
        if(fs > 0.0f) { sum_fs += fs; sum_wo += fs * rs[i][j]; }
    }

    float delta_u = (sum_fs > 0.0f) ? sum_wo / sum_fs : 0.0f;
    float delta_pwm = delta_u * Gu;

    float output;
    if(useIntegration) {
        // PI mode: integrate the correction (for speed/traction)
        integrated_output = constrain(integrated_output + delta_pwm, -255.0f, 255.0f);
        output = integrated_output;
    } else {
        // PD mode: direct proportional output (for position/steering)
        output = constrain(delta_pwm, -255.0f, 255.0f);
    }

    lastState.error      = error;
    lastState.d_error    = d_error;
    lastState.error_norm = en;
    lastState.d_error_norm = den;
    lastState.delta_u    = delta_u;
    lastState.delta_pwm  = delta_pwm;
    lastState.final_pwm  = output;

    prev_error = error;
    return output;
}

void FuzzyController::reset() {
    prev_error = 0.0f;
    integrated_output = 0.0f;
    memset(&lastState, 0, sizeof(FuzzyState));
}

void FuzzyController::setGains(float ge, float gde, float gu) {
    Ge = ge; Gde = gde; Gu = gu;
}
