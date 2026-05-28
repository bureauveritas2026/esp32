#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_arduino_version.h>
#include <Preferences.h>
#include "PinDefinitions.h"
#include "Config.h"
#include "FuzzyController.h"

// --- LEDC Compatibility (Core v2 vs v3) ---
void setupLEDC(uint8_t pin, uint32_t freq, uint8_t res, uint8_t ch) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pin, freq, res);
#else
    ledcSetup(ch, freq, res);
    ledcAttachPin(pin, ch);
#endif
}
void writeLEDC(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, duty);
#else
    ledcWrite(ch, duty);
#endif
}

// --- Global Objects ---
WiFiClient   espClient;
PubSubClient mqttClient(espClient);
Preferences  prefs; // Preferences object for persistent hardware calibration

// Traction: Fuzzy PI (velocity control, rear motor)
// Steering: Fuzzy PD (position control, front motor)
// Constructor: (Ge, Gde, Gu, integrate, minPwm, deadZoneThreshold)
// Ge: error normalizer — lower = more fuzzy resolution in working range
// Gde: derivative normalizer — note: d_error is now normalized by dt in the controller
// Gu: output scaler — higher = faster response
// minPwm: minimum PWM to overcome N20 motor static friction (~55 for TB6612FNG)
// deadZoneThreshold: error below which dead-zone compensation is disabled
// Traction PI: Ge=error norm, Gde=d_error norm, Gu=output scale, integrate=true
// Steering PD PURE: Gu must be HIGH because output is NOT accumulated — direct proportional action
// With Gu=220, a normalized error of 1.0 → delta_pwm = 220 (full drive), error of 0.1 → 22 PWM (fine control)
FuzzyController fuzzyTrac(0.008f, 0.002f, 30.0f, true,  55.0f, 3.0f);
FuzzyController fuzzySteer(0.01f,  0.004f, 220.0f, false, 60.0f, 4.0f);

// --- Encoder State ---
volatile long encTrac  = 0;
volatile long encSteer = 0;

long lastEncTrac = 0;

// --- Dynamic Hardware Calibration Polarities (NVS) ---
volatile int tracMotorDir  = 1;
volatile int tracEncDir    = 1;
volatile int steerMotorDir = 1;
volatile int steerEncDir   = 1;

// --- Control Targets ---
float targetSpeed  = 0.0f;  // ticks/sec for traction motor
long  targetAngle  = 0;     // encoder ticks for steering angle

// Steering software limits (prevents physical over-rotation, 826 ticks = 360 degrees)
const long STEER_MAX = 826;

// --- Output Values ---
float currentPwmTrac  = 0.0f;
float currentPwmSteer = 0.0f;

// --- Timing ---
unsigned long lastControlTime   = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long CONTROL_MS = 50; // 20 Hz

// --- MQTT Topics ---
String clientId, topicTelemetry, topicCommand, topicConfig, macStr;

// --- Precision Arrival Control State Machine ---
bool  isArrivalActive     = false;
int   arrivalPhase        = 0;      // 0: Idle, 1: Steering orientation, 2: Traction translation
float targetDistanceCm    = 0.0f;
float targetAngleDeg      = 0.0f;
long  targetAngleTicks    = 0;
long  targetTracTicks     = 0;
long  startTracPos        = 0;
float remainingDistCm     = 0.0f;

// Closed loop velocity profiling constants
const float arrivalKpTrac      = 1.2f;   // Speed target = Kp * remaining ticks (higher = faster start)
const float maxArrivalSpeed    = 200.0f; // Limit target speed during arrival
const float arrivalTolerance   = 10.0f;  // Position reached threshold (~4mm)
const float steerTolerance     = 5.0f;   // Steering alignment threshold (~2.2 deg)
const float steerVelTolerance  = 8.0f;   // Max angular velocity (ticks/s) when declaring alignment stable
int steeringStableCount        = 0;      // Consecutive loops the steering must be stable before moving

// ---- ISRs ----
void IRAM_ATTR isrTrac() {
    encTrac = encTrac + (tracEncDir * (digitalRead(PIN_ENC_TRAC_B) == HIGH ? 1 : -1));
}
void IRAM_ATTR isrSteer() {
    encSteer = encSteer + (steerEncDir * (digitalRead(PIN_ENC_STEER_B) == HIGH ? 1 : -1));
}

// ---- Motor Write ----
void setTraction(int pwm) {
    // Apply physical motor polarity inversion dynamically
    pwm = pwm * tracMotorDir;

    if (pwm > 0) {
        digitalWrite(PIN_TRAC_IN1, HIGH); digitalWrite(PIN_TRAC_IN2, LOW);
    } else if (pwm < 0) {
        digitalWrite(PIN_TRAC_IN1, LOW);  digitalWrite(PIN_TRAC_IN2, HIGH);
    } else {
        digitalWrite(PIN_TRAC_IN1, LOW);  digitalWrite(PIN_TRAC_IN2, LOW);
    }
    writeLEDC(PIN_PWM_TRAC, PWM_CH_TRAC, constrain(abs(pwm), 0, 255));
}

void setSteering(int pwm) {
    // Apply software end-stop limits
    noInterrupts();
    long pos = encSteer;
    interrupts();
    if ((pos >= STEER_MAX && pwm > 0) || (pos <= -STEER_MAX && pwm < 0)) pwm = 0;

    // Apply physical motor polarity inversion dynamically
    pwm = pwm * steerMotorDir;

    if (pwm > 0) {
        digitalWrite(PIN_STEER_IN1, HIGH); digitalWrite(PIN_STEER_IN2, LOW);
    } else if (pwm < 0) {
        digitalWrite(PIN_STEER_IN1, LOW);  digitalWrite(PIN_STEER_IN2, HIGH);
    } else {
        digitalWrite(PIN_STEER_IN1, LOW);  digitalWrite(PIN_STEER_IN2, LOW);
    }
    writeLEDC(PIN_PWM_STEER, PWM_CH_STEER, constrain(abs(pwm), 0, 255));
}

// ---- JSON value parser (no external library) ----
float parseVal(String &p, const char* key) {
    int idx = p.indexOf(key);
    if (idx == -1) return 0.0f;
    int c = p.indexOf(':', idx);
    if (c == -1) return 0.0f;
    int e1 = p.indexOf(',', c), e2 = p.indexOf('}', c);
    int e  = (e1 != -1 && e1 < e2) ? e1 : e2;
    String v = p.substring(c + 1, e); v.trim();
    return v.toFloat();
}

// ---- MQTT Callback ----
void mqttCallback(char* topic, byte* payload, unsigned int len) {
    String p = "";
    for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
    Serial.println("CMD: " + p);

    if (p.indexOf("\"stop\"") != -1) {
        isArrivalActive = false;
        arrivalPhase = 0;
        targetSpeed = 0; targetAngle = 0;
        fuzzyTrac.reset(); fuzzySteer.reset();
        setTraction(0); setSteering(0);
        remainingDistCm = 0.0f;
    } else if (p.indexOf("\"drive\"") != -1) {
        // Intercepted by manual override: cancels precision arrival control
        isArrivalActive = false;
        arrivalPhase = 0;
        remainingDistCm = 0.0f;
        targetSpeed = parseVal(p, "\"speed\"");
        targetAngle = (long)parseVal(p, "\"angle\"");
        targetAngle = constrain(targetAngle, -STEER_MAX, STEER_MAX);
    } else if (p.indexOf("\"goto\"") != -1) {
        float distCm = parseVal(p, "\"dist\"");
        float angleDeg = parseVal(p, "\"angle\"");

        targetDistanceCm = distCm;
        targetAngleDeg = constrain(angleDeg, -360.0f, 360.0f);

        // Convert target angle and distance to encoder ticks
        targetAngleTicks = (long)(targetAngleDeg * (826.0f / 360.0f));
        targetTracTicks = (long)(targetDistanceCm * TICKS_PER_CM);

        noInterrupts();
        startTracPos = encTrac;
        interrupts();

        isArrivalActive = true;
        arrivalPhase = 1; // start with Phase 1 (Steering)
        steeringStableCount = 0;
        fuzzyTrac.reset();
        fuzzySteer.reset();
        Serial.printf("Arrival Triggered: Dist=%.1f cm (%ld ticks), Angle=%.1f deg (%ld ticks)\n",
                      targetDistanceCm, targetTracTicks, targetAngleDeg, targetAngleTicks);

    } else if (p.indexOf("\"goto_pt\"") != -1) {
        float x_m = parseVal(p, "\"x\"");
        float y_m = parseVal(p, "\"y\"");

        // Pure Pursuit / geometric arc calculations in local coordinate system
        float L = WHEELBASE_M;
        float R = 0.0f;
        float angleDeg = 0.0f;
        float dist_m = 0.0f;

        if (fabsf(x_m) < 0.001f) {
            // Straight line motion
            dist_m = y_m;
            angleDeg = 0.0f;
        } else {
            R = (x_m * x_m + y_m * y_m) / (2.0f * x_m);
            float angleRad = atan(L / R);
            angleDeg = angleRad * 180.0f / PI;

            // Chord length distance estimation with sign for forward/backward
            float chord = sqrt(x_m * x_m + y_m * y_m);
            dist_m = (y_m < 0.0f) ? -chord : chord;

            if (y_m < 0.0f) {
                angleDeg = -angleDeg; // Steering polarity inversion for reverse gear
            }
        }

        targetDistanceCm = dist_m * 100.0f;
        targetAngleDeg = constrain(angleDeg, -360.0f, 360.0f);

        // Convert target angle and distance to encoder ticks
        targetAngleTicks = (long)(targetAngleDeg * (826.0f / 360.0f));
        targetTracTicks = (long)(targetDistanceCm * TICKS_PER_CM);

        noInterrupts();
        startTracPos = encTrac;
        interrupts();

        isArrivalActive = true;
        arrivalPhase = 1; // start with Phase 1 (Steering)
        steeringStableCount = 0;
        fuzzyTrac.reset();
        fuzzySteer.reset();
        Serial.printf("Arrival Point Triggered: X=%.2f m, Y=%.2f m -> Calc Dist=%.1f cm (%ld ticks), Angle=%.1f deg (%ld ticks)\n",
                      x_m, y_m, targetDistanceCm, targetTracTicks, targetAngleDeg, targetAngleTicks);

    } else if (p.indexOf("\"tune\"") != -1) {
        float ge = parseVal(p, "\"ge\""), gde = parseVal(p, "\"gde\""), gu = parseVal(p, "\"gu\"");
        if (p.indexOf("\"motor\":\"trac\"") != -1)  fuzzyTrac.setGains(ge, gde, gu);
        if (p.indexOf("\"motor\":\"steer\"") != -1) fuzzySteer.setGains(ge, gde, gu);
    } else if (p.indexOf("\"polarity\"") != -1) {
        int t_mot = (int)parseVal(p, "\"t_mot\"");
        int t_enc = (int)parseVal(p, "\"t_enc\"");
        int s_mot = (int)parseVal(p, "\"s_mot\"");
        int s_enc = (int)parseVal(p, "\"s_enc\"");

        if (t_mot == 1 || t_mot == -1)   tracMotorDir  = t_mot;
        if (t_enc == 1 || t_enc == -1)   tracEncDir    = t_enc;
        if (s_mot == 1 || s_mot == -1)   steerMotorDir = s_mot;
        if (s_enc == 1 || s_enc == -1)   steerEncDir   = s_enc;

        prefs.begin("polarity", false);
        prefs.putInt("trac_mot", (int)tracMotorDir);
        prefs.putInt("trac_enc", (int)tracEncDir);
        prefs.putInt("steer_mot", (int)steerMotorDir);
        prefs.putInt("steer_enc", (int)steerEncDir);
        prefs.end();

        Serial.printf("POLARIDAD ACTUALIZADA: TracMot=%d, TracEnc=%d, SteerMot=%d, SteerEnc=%d\n",
                      tracMotorDir, tracEncDir, steerMotorDir, steerEncDir);
    } else if (p.indexOf("\"reset\"") != -1) {
        isArrivalActive = false;
        arrivalPhase = 0;
        noInterrupts(); encTrac = 0; encSteer = 0; interrupts();
        lastEncTrac = 0;
        fuzzyTrac.reset(); fuzzySteer.reset();
        remainingDistCm = 0.0f;
    }
}

// ---- WiFi ----
void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi");
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500); Serial.print('.');
    }
    Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " FAIL");
}

// ---- MQTT Reconnect ----
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("MQTT...");
        if (mqttClient.connect(clientId.c_str())) {
            mqttClient.subscribe(topicCommand.c_str());
            mqttClient.subscribe(topicConfig.c_str());
            Serial.println("OK");
        } else {
            Serial.print("ERR "); Serial.println(mqttClient.state());
            delay(5000);
        }
    }
}

// ---- Setup ----
void setup() {
    Serial.begin(115200);

    // Iniciar Preferences y cargar calibración de hardware guardada
    prefs.begin("polarity", false);
    tracMotorDir  = prefs.getInt("trac_mot", 1);
    tracEncDir    = prefs.getInt("trac_enc", 1);
    steerMotorDir = prefs.getInt("steer_mot", 1);
    steerEncDir   = prefs.getInt("steer_enc", 1);
    prefs.end();

    // Validar integridad de las polaridades cargadas (solo 1 o -1)
    if (tracMotorDir != 1 && tracMotorDir != -1)   tracMotorDir = 1;
    if (tracEncDir != 1 && tracEncDir != -1)       tracEncDir = 1;
    if (steerMotorDir != 1 && steerMotorDir != -1) steerMotorDir = 1;
    if (steerEncDir != 1 && steerEncDir != -1)     steerEncDir = 1;

    pinMode(PIN_TRAC_IN1,  OUTPUT); pinMode(PIN_TRAC_IN2,  OUTPUT);
    pinMode(PIN_STEER_IN1, OUTPUT); pinMode(PIN_STEER_IN2, OUTPUT);

    setupLEDC(PIN_PWM_TRAC,  PWM_FREQ, PWM_RES, PWM_CH_TRAC);
    setupLEDC(PIN_PWM_STEER, PWM_FREQ, PWM_RES, PWM_CH_STEER);
    setTraction(0); setSteering(0);

    // GPIO 34 and 35 are input-only and do not have internal pull-up/down resistors.
    // They must use external pull-ups (configured here as standard INPUT).
    pinMode(PIN_ENC_TRAC_A,  INPUT);        pinMode(PIN_ENC_TRAC_B,  INPUT);
    pinMode(PIN_ENC_STEER_A, INPUT_PULLUP); pinMode(PIN_ENC_STEER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_TRAC_A),  isrTrac,  RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_STEER_A), isrSteer, RISING);

    uint8_t mac[6]; WiFi.macAddress(mac);
    char ms[13];
    snprintf(ms, sizeof(ms), "%02X%02X%02X%02X%02X%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    macStr        = String(ms);
    clientId      = "ESP32Ack_" + macStr;
    topicTelemetry = String(MQTT_BASE_TOPIC) + macStr + "/telemetry";
    topicCommand   = String(MQTT_BASE_TOPIC) + macStr + "/command";
    topicConfig    = String(MQTT_BASE_TOPIC) + macStr + "/config";

    Serial.println();
    Serial.println("==================================================");
    Serial.println("===       Inicializando Robot Ackermann        ===");
    Serial.print("ESP32 MAC (con dos puntos): ");
    Serial.println(WiFi.macAddress());
    Serial.print("ESP32 MAC (formato plano):  ");
    Serial.println(macStr);
    Serial.print("MQTT Broker:                ");
    Serial.println(MQTT_BROKER);
    Serial.print("Telemetry Topic:            ");
    Serial.println(topicTelemetry);
    Serial.print("Command Topic:              ");
    Serial.println(topicCommand);
    Serial.println("==================================================");
    Serial.println();

    setupWiFi();
    mqttClient.setBufferSize(1024); // Increase buffer size to 1024 to support large telemetry JSON packets
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    lastControlTime = lastTelemetryTime = millis();
}

// ---- Loop ----
void loop() {
    // WiFi watchdog
    if (WiFi.status() != WL_CONNECTED) {
        static unsigned long wt = 0;
        if (millis() - wt > 10000) { wt = millis(); WiFi.begin(WIFI_SSID, WIFI_PASSWORD); }
    } else {
        if (!mqttClient.connected()) reconnectMQTT();
        mqttClient.loop();
    }

    unsigned long now = millis();

    // --- Control Loop (20 Hz) ---
    if (now - lastControlTime >= CONTROL_MS) {
        float dt = (now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        noInterrupts();
        long tTrac  = encTrac;
        long tSteer = encSteer;
        interrupts();

        // Traction: velocity (ticks/s)
        float speedTrac = (float)(tTrac - lastEncTrac) / dt;
        lastEncTrac = tTrac;

        if (isArrivalActive) {
            // ==========================================
            // HIGH-PRECISION CLOSED-LOOP ARRIVAL SYSTEM
            // ==========================================
            if (arrivalPhase == 1) {
                // --- PHASE 1: Steering Alignment (Traction Stopped) ---
                targetSpeed = 0.0f;
                currentPwmTrac = fuzzyTrac.compute(0.0f, speedTrac, dt);
                setTraction((int)currentPwmTrac); // enforce 0 speed in closed loop

                // Lock steering to calculated target ticks
                currentPwmSteer = fuzzySteer.compute((float)targetAngleTicks, (float)tSteer, dt);
                setSteering((int)currentPwmSteer);

                // Check stabilization: error within tolerance AND motor is nearly stationary
                float errorSteer = (float)targetAngleTicks - (float)tSteer;
                // d_error here is the true angular velocity (ticks/s) since the PD controller
                // normalizes by dt internally — use lastState.d_error as raw velocity reference
                float steerVel = fuzzySteer.lastState.d_error;

                if (fabsf(errorSteer) <= steerTolerance && fabsf(steerVel) < steerVelTolerance) {
                    steeringStableCount++;
                    if (steeringStableCount >= 4) { // 4 consecutive samples = 200ms stability
                        arrivalPhase = 2;
                        setSteering(0); // hard-stop steering before switching
                        noInterrupts();
                        startTracPos = encTrac;
                        interrupts();
                        fuzzyTrac.reset(); // clear integrator for perfect translation start
                        steeringStableCount = 0;
                        Serial.printf("Arrival Phase 1 OK → Steer error=%.1f ticks, vel=%.1f ticks/s\n",
                                      errorSteer, steerVel);
                        Serial.println("Starting Phase 2 (Translation).");
                    }
                } else {
                    steeringStableCount = 0; // reset if any instability
                }
            }
            else if (arrivalPhase == 2) {
                // --- PHASE 2: Traction Translation (Steering Locked) ---
                // Keep steering wheels locked at the target angle
                currentPwmSteer = fuzzySteer.compute((float)targetAngleTicks, (float)tSteer, dt);
                setSteering((int)currentPwmSteer);

                // Calculate relative traveled distance and error
                long traveledTicks = tTrac - startTracPos;
                long remainingTicks = targetTracTicks - traveledTicks;
                remainingDistCm = (float)remainingTicks / TICKS_PER_CM;

                // Cascaded speed profiling: proportional ramp down as we approach target.
                // Minimum speed 25 ticks/s to ensure motor always overcomes friction until very close.
                float speedTarget = arrivalKpTrac * (float)remainingTicks;
                speedTarget = constrain(speedTarget, -maxArrivalSpeed, maxArrivalSpeed);

                // Dead-band: below 12 ticks/s target (fine approach zone) step down gracefully
                if (fabsf(speedTarget) < 12.0f && fabsf(speedTarget) > 0.0f) {
                    speedTarget = (speedTarget > 0.0f) ? 12.0f : -12.0f;
                }
                // Hard stop zone — already within tolerance
                if (abs(remainingTicks) <= (long)arrivalTolerance) {
                    speedTarget = 0.0f;
                }

                currentPwmTrac = fuzzyTrac.compute(speedTarget, speedTrac, dt);
                setTraction((int)currentPwmTrac);

                // Check arrival tolerance and zero speed to stop
                if (abs(remainingTicks) <= arrivalTolerance && fabsf(speedTrac) < 5.0f) {
                    isArrivalActive = false;
                    arrivalPhase = 0;
                    targetSpeed = 0.0f;
                    targetAngle = 0;
                    fuzzyTrac.reset();
                    fuzzySteer.reset();
                    setTraction(0);
                    setSteering(0);
                    remainingDistCm = 0.0f;
                    Serial.println("Arrival: Target reached successfully. Motors stopped.");
                }
            }
        } else {
            // ==========================================
            // NORMAL MANUAL TELEOPERATION SYSTEM
            // ==========================================
            currentPwmTrac = fuzzyTrac.compute(targetSpeed, speedTrac, dt);
            setTraction((int)currentPwmTrac);

            currentPwmSteer = fuzzySteer.compute((float)targetAngle, (float)tSteer, dt);
            setSteering((int)currentPwmSteer);
        }
    }

    // --- Telemetry (configurable interval) ---
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS && mqttClient.connected()) {
        lastTelemetryTime = now;

        noInterrupts();
        long posTrac = encTrac; long posSteer = encSteer;
        interrupts();

        // Accurate speed snapshot: ticks since last telemetry / elapsed seconds
        static long lastPosTelTrac = 0;
        float spdSnap = (float)(posTrac - lastPosTelTrac) / (TELEMETRY_INTERVAL_MS / 1000.0f);
        lastPosTelTrac = posTrac;

        char buf[1024];
        snprintf(buf, sizeof(buf),
            "{\"connected\":true,"
             "\"trac\":{\"target\":%.1f,\"speed\":%.1f,\"pos\":%ld,\"pwm\":%.1f,"
                        "\"err\":%.2f,\"derr\":%.2f,\"du\":%.2f},"
             "\"steer\":{\"target\":%ld,\"pos\":%ld,\"pwm\":%.1f,"
                         "\"err\":%.2f,\"derr\":%.2f,\"du\":%.2f},"
             "\"arrival\":{\"active\":%s,\"phase\":%d,\"target_dist\":%.1f,\"remaining_dist\":%.1f,\"target_angle\":%.1f},"
             "\"polarity\":{\"t_mot\":%d,\"t_enc\":%d,\"s_mot\":%d,\"s_enc\":%d},"
             "\"fuzzy\":{"
               "\"trac_mu_e\":[%.2f,%.2f,%.2f,%.2f,%.2f],"
               "\"trac_mu_de\":[%.2f,%.2f,%.2f],"
               "\"steer_mu_e\":[%.2f,%.2f,%.2f,%.2f,%.2f],"
               "\"steer_mu_de\":[%.2f,%.2f,%.2f]"
             "}}",
            targetSpeed, spdSnap, posTrac, currentPwmTrac,
            fuzzyTrac.lastState.error, fuzzyTrac.lastState.d_error, fuzzyTrac.lastState.delta_pwm,

            targetAngle, posSteer, currentPwmSteer,
            fuzzySteer.lastState.error, fuzzySteer.lastState.d_error, fuzzySteer.lastState.delta_pwm,

            isArrivalActive ? "true" : "false", arrivalPhase, targetDistanceCm, remainingDistCm, targetAngleDeg,
            tracMotorDir, tracEncDir, steerMotorDir, steerEncDir,

            fuzzyTrac.lastState.mu_e[0],  fuzzyTrac.lastState.mu_e[1],
            fuzzyTrac.lastState.mu_e[2],  fuzzyTrac.lastState.mu_e[3],  fuzzyTrac.lastState.mu_e[4],
            fuzzyTrac.lastState.mu_de[0], fuzzyTrac.lastState.mu_de[1], fuzzyTrac.lastState.mu_de[2],

            fuzzySteer.lastState.mu_e[0],  fuzzySteer.lastState.mu_e[1],
            fuzzySteer.lastState.mu_e[2],  fuzzySteer.lastState.mu_e[3], fuzzySteer.lastState.mu_e[4],
            fuzzySteer.lastState.mu_de[0], fuzzySteer.lastState.mu_de[1],fuzzySteer.lastState.mu_de[2]
        );
        mqttClient.publish(topicTelemetry.c_str(), buf);
    }

    // --- Broadcast a Discovery ping every 10 seconds ---
    static unsigned long lastDiscoveryTime = 0;
    if (now - lastDiscoveryTime >= 10000 && mqttClient.connected()) {
        lastDiscoveryTime = now;
        char discBuf[128];
        snprintf(discBuf, sizeof(discBuf), "{\"mac\":\"%s\",\"status\":\"online\",\"ip\":\"%s\"}", macStr.c_str(), WiFi.localIP().toString().c_str());
        mqttClient.publish("esp32/robot_fuzzy/discovery", discBuf);
    }
}
