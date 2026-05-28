#include "FuzzyController.h"

FuzzyController::FuzzyController(float ge, float gde, float gu) {
    Ge = ge;
    Gde = gde;
    Gu = gu;
    prev_error = 0.0f;
    integrated_output = 0.0f;
    memset(&lastState, 0, sizeof(FuzzyState));
}

// Triangular membership function
float FuzzyController::trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x == b) return 1.0f;
    if (x < b) {
        return (x - a) / (b - a);
    } else {
        return (c - x) / (c - b);
    }
}

// Trapezoidal membership function
float FuzzyController::trapmf(float x, float a, float b, float c, float d) {
    if (x <= a || x >= d) return 0.0f;
    if (x >= b && x <= c) return 1.0f;
    if (x < b) {
        return (x - a) / (b - a);
    } else {
        return (d - x) / (d - c);
    }
}

float FuzzyController::compute(float target, float current, float dt) {
    // 1. Calculate error and change of error
    float error = target - current;
    float d_error = error - prev_error; // Gain Gde will absorb dt if loop frequency is constant

    // 2. Normalize inputs to [-1.0, 1.0]
    float error_norm = constrain(error * Ge, -1.0f, 1.0f);
    float d_error_norm = constrain(d_error * Gde, -1.0f, 1.0f);

    // 3. Fuzzify Input 1: Normalized Error (e_N)
    // NB (Negative Big): Left shoulder
    float mu_e_NB = trapmf(error_norm, -2.0f, -2.0f, -0.6f, -0.2f);
    // NS (Negative Small): Triangle
    float mu_e_NS = trimf(error_norm, -0.5f, -0.25f, 0.0f);
    // ZE (Zero): Triangle
    float mu_e_ZE = trimf(error_norm, -0.2f, 0.0f, 0.2f);
    // PS (Positive Small): Triangle
    float mu_e_PS = trimf(error_norm, 0.0f, 0.25f, 0.5f);
    // PB (Positive Big): Right shoulder
    float mu_e_PB = trapmf(error_norm, 0.2f, 0.6f, 2.0f, 2.0f);

    // 4. Fuzzify Input 2: Normalized Change of Error (de_N)
    // N (Negative): Left shoulder
    float mu_de_N = trapmf(d_error_norm, -2.0f, -2.0f, -0.4f, 0.0f);
    // ZE (Zero): Triangle
    float mu_de_ZE = trimf(d_error_norm, -0.4f, 0.0f, 0.4f);
    // P (Positive): Right shoulder
    float mu_de_P = trapmf(d_error_norm, 0.0f, 0.4f, 2.0f, 2.0f);

    // Store membership values for telemetry
    lastState.mu_e[0] = mu_e_NB;
    lastState.mu_e[1] = mu_e_NS;
    lastState.mu_e[2] = mu_e_ZE;
    lastState.mu_e[3] = mu_e_PS;
    lastState.mu_e[4] = mu_e_PB;

    lastState.mu_de[0] = mu_de_N;
    lastState.mu_de[1] = mu_de_ZE;
    lastState.mu_de[2] = mu_de_P;

    // 5. Rule Base & Inference (Mamdani Min composition)
    // Rule output singletons (defined for 5 error levels x 3 d_error levels)
    // Output singletons: NB=-1.0, NM=-0.6, NS=-0.3, ZE=0.0, PS=0.3, PM=0.6, PB=1.0
    const float rule_singletons[5][3] = {
        // de_N: N,      ZE,     P
        { -1.0f,  -1.0f,  -0.6f }, // e_N: NB
        { -0.6f,  -0.3f,   0.0f }, // e_N: NS
        { -0.3f,   0.0f,   0.3f }, // e_N: ZE
        {  0.0f,   0.3f,   0.6f }, // e_N: PS
        {  0.6f,   1.0f,   1.0f }  // e_N: PB
    };

    float sum_firing_strength = 0.0f;
    float sum_weighted_output = 0.0f;

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 3; j++) {
            // Firing strength of rule (i, j) using MIN operator
            float firing_strength = min(lastState.mu_e[i], lastState.mu_de[j]);
            if (firing_strength > 0.0f) {
                sum_firing_strength += firing_strength;
                sum_weighted_output += firing_strength * rule_singletons[i][j];
            }
        }
    }

    // 6. Defuzzification (Weighted Average / Centroid of Singletons)
    float delta_u = 0.0f;
    if (sum_firing_strength > 0.0f) {
        delta_u = sum_weighted_output / sum_firing_strength;
    }

    // 7. Output Scaling & Integration
    float delta_pwm = delta_u * Gu;
    integrated_output += delta_pwm;
    integrated_output = constrain(integrated_output, -255.0f, 255.0f);

    // Save outputs to state structure
    lastState.error = error;
    lastState.d_error = d_error;
    lastState.error_norm = error_norm;
    lastState.d_error_norm = d_error_norm;
    lastState.delta_u = delta_u;
    lastState.delta_pwm = delta_pwm;
    lastState.final_pwm = integrated_output;

    // 8. Update state for next cycle
    prev_error = error;

    return integrated_output;
}

void FuzzyController::reset() {
    prev_error = 0.0f;
    integrated_output = 0.0f;
    memset(&lastState, 0, sizeof(FuzzyState));
}

void FuzzyController::setGains(float ge, float gde, float gu) {
    Ge = ge;
    Gde = gde;
    Gu = gu;
}
