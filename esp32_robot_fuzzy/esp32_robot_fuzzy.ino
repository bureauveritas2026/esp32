// =============================================================
// Fuzzy Robot ESP32 — Firmware v4.0
// Motor Tracción (TRASERO): Control de VELOCIDAD con Fuzzy PI
// Motor Dirección (DELANTERO): Control de POSICIÓN con PD clásico
// Límite físico de dirección: ±826 ticks = ±360°, BLOQUEADO
// =============================================================

#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_arduino_version.h>
#include <Preferences.h>

#include "PinDefinitions.h"
#include "Config.h"
#include "FuzzyController.h"

// --------------------------------------------------
// LEDC Compat (Core v2 / v3)
// --------------------------------------------------
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

// --------------------------------------------------
// MQTT / Preferences
// --------------------------------------------------
WiFiClient   espClient;
PubSubClient mqttClient(espClient);
Preferences  prefs;

// --------------------------------------------------
// CONTROLADORES
// Tracción: Fuzzy PI (velocidad) — acumula delta_pwm
// Dirección: NO usa FuzzyController — PD clásico propio (más estable)
// --------------------------------------------------
FuzzyController fuzzyTrac(
    0.008f,  // Ge  — normalizar error de velocidad (~125 ticks/s rango)
    0.002f,  // Gde — normalizar derivada
    30.0f,   // Gu  — escala de salida
    true,    // integrate = PI
    55.0f,   // minPwm — umbral fricción estática N20
    3.0f     // deadZoneThreshold
);

// --------------------------------------------------
// PD CLÁSICO PARA DIRECCIÓN (posición)
// Más predecible que fuzzy para control de posición
// Kp: ganancia proporcional (en ticks de error → PWM)
// Kd: ganancia derivativa   (velocidad angular → PWM)
// --------------------------------------------------
const float STEER_KP      = 0.55f;  // ~55 PWM por 100 ticks de error
const float STEER_KD      = 0.18f;  // amortiguamiento
const float STEER_MIN_PWM = 55.0f;  // mínimo para vencer fricción estática
const float STEER_DEAD_ZONE = 5.0f; // ticks — zona muerta (motor apagado)
float steerPrevError = 0.0f;        // derivada del PD

// --------------------------------------------------
// ENCODERS
// --------------------------------------------------
volatile long encTrac  = 0;
volatile long encSteer = 0;
long lastEncTrac = 0;

// --------------------------------------------------
// POLARIDADES DINÁMICAS (NVS)
// --------------------------------------------------
volatile int tracMotorDir  = 1;
volatile int tracEncDir    = 1;
volatile int steerMotorDir = 1;
volatile int steerEncDir   = 1;
volatile bool swapHardware = false; // si true: cruzar salidas trac↔steer

// --------------------------------------------------
// OBJETIVOS DE CONTROL
// --------------------------------------------------
float targetSpeed = 0.0f;   // ticks/s para tracción
long  targetAngle = 0;      // ticks de encoder para dirección (abs.)

// LÍMITE FÍSICO DE DIRECCIÓN: ±826 ticks = ±360°
// BLOQUEADO EN HARDWARE — no puede sobrepasar nunca
const long STEER_MAX = 826;

// --------------------------------------------------
// SALIDAS PWM
// --------------------------------------------------
float currentPwmTrac  = 0.0f;
float currentPwmSteer = 0.0f;

// --------------------------------------------------
// TEMPORIZADORES
// --------------------------------------------------
unsigned long lastControlTime   = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long CONTROL_MS  = 50; // 20 Hz

// --------------------------------------------------
// MQTT TOPICS
// --------------------------------------------------
String clientId, topicTelemetry, topicCommand, topicConfig, macStr;

// --------------------------------------------------
// SISTEMA DE LLEGADA PRECISA (autopilot)
// --------------------------------------------------
bool  isArrivalActive  = false;
int   arrivalPhase     = 0;
float targetDistanceCm = 0.0f;
float targetAngleDeg   = 0.0f;
long  targetAngleTicks = 0;
long  targetTracTicks  = 0;
long  startTracPos     = 0;
float remainingDistCm  = 0.0f;

const float arrivalKpTrac    = 0.8f;
const float maxArrivalSpeed  = 150.0f;
const float arrivalTolerance = 8.0f;
const float steerTolerance   = 6.0f;   // ticks (~2.6°)
int   steeringStableCount    = 0;

// --------------------------------------------------
// MODO DIAGNÓSTICO RAW
// Aplica PWM directo durante 3 segundos, saltándose el control difuso
// Para verificar hardware (cables cruzados, encoders, etc.)
// --------------------------------------------------
bool          isDiagActive = false;
String        diagMotor    = "";
int           diagPwm      = 0;
unsigned long diagStartTime = 0;
const unsigned long DIAG_DURATION_MS = 3000;

// --------------------------------------------------
// ISRs — ENCODERS CUADRATURA
// --------------------------------------------------
void IRAM_ATTR isrTrac() {
    if (!swapHardware) {
        encTrac = encTrac + (tracEncDir * (digitalRead(PIN_ENC_TRAC_B) == HIGH ? 1 : -1));
    } else {
        // Swap: el encoder que físicamente es TRAC se usa como STEER
        encSteer = encSteer + (steerEncDir * (digitalRead(PIN_ENC_TRAC_B) == HIGH ? 1 : -1));
    }
}
void IRAM_ATTR isrSteer() {
    if (!swapHardware) {
        encSteer = encSteer + (steerEncDir * (digitalRead(PIN_ENC_STEER_B) == HIGH ? 1 : -1));
    } else {
        // Swap: el encoder que físicamente es STEER se usa como TRAC
        encTrac = encTrac + (tracEncDir * (digitalRead(PIN_ENC_STEER_B) == HIGH ? 1 : -1));
    }
}

// --------------------------------------------------
// APLICAR MOTOR TRACCIÓN (trasero)
// pwm: negativo = retroceder, positivo = avanzar, 0 = freno
// --------------------------------------------------
void applyTrac(int pwm) {
    uint8_t in1, in2, pwmPin, ch;

    if (!swapHardware) {
        // Hardware normal: TRAC → pines de TRAC
        pwm = pwm * tracMotorDir;
        in1 = PIN_TRAC_IN1; in2 = PIN_TRAC_IN2;
        pwmPin = PIN_PWM_TRAC; ch = PWM_CH_TRAC;
    } else {
        // Swap: señal de TRAC → pines físicos de STEER
        pwm = pwm * steerMotorDir;
        in1 = PIN_STEER_IN1; in2 = PIN_STEER_IN2;
        pwmPin = PIN_PWM_STEER; ch = PWM_CH_STEER;
    }

    if      (pwm > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
    else if (pwm < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
    else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
    writeLEDC(pwmPin, ch, constrain(abs(pwm), 0, 255));
}

// --------------------------------------------------
// APLICAR MOTOR DIRECCIÓN (delantero)
// BLOQUEADO: si pos >= STEER_MAX y se pide girar en esa dirección → PWM=0
// --------------------------------------------------
void applySteer(int pwm) {
    // *** BLOQUEO FÍSICO — NUNCA SOBREPASAR ±826 TICKS ***
    noInterrupts();
    long pos = encSteer;
    interrupts();

    if (pos >= STEER_MAX && pwm > 0) pwm = 0;
    if (pos <= -STEER_MAX && pwm < 0) pwm = 0;

    uint8_t in1, in2, pwmPin, ch;

    if (!swapHardware) {
        // Hardware normal: STEER → pines de STEER
        pwm = pwm * steerMotorDir;
        in1 = PIN_STEER_IN1; in2 = PIN_STEER_IN2;
        pwmPin = PIN_PWM_STEER; ch = PWM_CH_STEER;
    } else {
        // Swap: señal de STEER → pines físicos de TRAC
        pwm = pwm * tracMotorDir;
        in1 = PIN_TRAC_IN1; in2 = PIN_TRAC_IN2;
        pwmPin = PIN_PWM_TRAC; ch = PWM_CH_TRAC;
    }

    if      (pwm > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
    else if (pwm < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
    else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }
    writeLEDC(pwmPin, ch, constrain(abs(pwm), 0, 255));
}

// --------------------------------------------------
// CONTROL PD DE DIRECCIÓN — retorna PWM listo para applySteer()
// target: ticks objetivo, current: ticks actuales, dt: seg
// --------------------------------------------------
float computeSteerPD(long target, long current, float dt) {
    float error = (float)(target - current);

    // Zona muerta absoluta: motor apagado cuando está en posición
    if (fabsf(error) <= STEER_DEAD_ZONE) {
        steerPrevError = error;
        return 0.0f;
    }

    float dError = (dt > 0.001f) ? (error - steerPrevError) / dt : 0.0f;
    steerPrevError = error;

    float output = STEER_KP * error + STEER_KD * dError;
    output = constrain(output, -255.0f, 255.0f);

    // Compensación de fricción estática: si hay señal pero es muy pequeña, elevar al mínimo
    if (fabsf(output) > 0.5f && fabsf(output) < STEER_MIN_PWM) {
        output = (output > 0.0f) ? STEER_MIN_PWM : -STEER_MIN_PWM;
    }

    return output;
}

// --------------------------------------------------
// PARSER JSON SIMPLE
// --------------------------------------------------
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

// --------------------------------------------------
// PARADA TOTAL DE EMERGENCIA
// --------------------------------------------------
void fullStop() {
    targetSpeed = 0.0f;
    targetAngle = 0;
    isArrivalActive = false;
    arrivalPhase = 0;
    isDiagActive = false;
    fuzzyTrac.reset();
    steerPrevError = 0.0f;
    applyTrac(0);
    applySteer(0);
    currentPwmTrac = 0.0f;
    currentPwmSteer = 0.0f;
    remainingDistCm = 0.0f;
    steeringStableCount = 0;
}

// --------------------------------------------------
// CALLBACK MQTT
// --------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int len) {
    String p = "";
    for (unsigned int i = 0; i < len; i++) p += (char)payload[i];
    Serial.println("[CMD] " + p);

    // === STOP ===
    if (p.indexOf("\"stop\"") != -1) {
        fullStop();
    }

    // === DRIVE (manual: velocidad + ángulo absoluto en grados) ===
    else if (p.indexOf("\"drive\"") != -1) {
        isArrivalActive = false;
        arrivalPhase = 0;
        isDiagActive = false;
        remainingDistCm = 0.0f;

        // VELOCIDAD → motor trasero (ticks/s)
        targetSpeed = parseVal(p, "\"speed\"");

        // ÁNGULO → grados absolutos convertidos a ticks
        // Si envías 0° → vuelve al centro, 90° → 90° horario, -90° antihorario
        float angleDeg = parseVal(p, "\"angle\"");
        angleDeg = constrain(angleDeg, -360.0f, 360.0f);
        targetAngle = (long)(angleDeg * (826.0f / 360.0f));
        // Forzar dentro de límites seguros también aquí
        targetAngle = constrain(targetAngle, -STEER_MAX, STEER_MAX);
    }

    // === GOTO (autopilot: distancia + ángulo) ===
    else if (p.indexOf("\"goto\"") != -1) {
        float distCm   = parseVal(p, "\"dist\"");
        float angleDeg = parseVal(p, "\"angle\"");

        targetDistanceCm = distCm;
        targetAngleDeg   = constrain(angleDeg, -360.0f, 360.0f);
        targetAngleTicks = constrain((long)(targetAngleDeg * (826.0f / 360.0f)), -STEER_MAX, STEER_MAX);
        targetTracTicks  = (long)(targetDistanceCm * TICKS_PER_CM);

        noInterrupts(); startTracPos = encTrac; interrupts();

        isArrivalActive = true;
        arrivalPhase    = 1;
        steeringStableCount = 0;
        steerPrevError  = 0.0f;
        fuzzyTrac.reset();
        Serial.printf("[GOTO] Dist=%.1fcm (%ld ticks), Angle=%.1fdeg (%ld ticks)\n",
                      targetDistanceCm, targetTracTicks, targetAngleDeg, targetAngleTicks);
    }

    // === GOTO_PT (autopilot: coordenadas X, Y en metros) ===
    else if (p.indexOf("\"goto_pt\"") != -1) {
        float x_m = parseVal(p, "\"x\"");
        float y_m = parseVal(p, "\"y\"");

        float L = WHEELBASE_M, R = 0.0f, angleDeg = 0.0f, dist_m = 0.0f;

        if (fabsf(x_m) < 0.001f) {
            dist_m = y_m; angleDeg = 0.0f;
        } else {
            R = (x_m * x_m + y_m * y_m) / (2.0f * x_m);
            angleDeg = atan(L / R) * 180.0f / PI;
            float chord = sqrt(x_m * x_m + y_m * y_m);
            dist_m = (y_m < 0.0f) ? -chord : chord;
            if (y_m < 0.0f) angleDeg = -angleDeg;
        }

        targetDistanceCm = dist_m * 100.0f;
        targetAngleDeg   = constrain(angleDeg, -360.0f, 360.0f);
        targetAngleTicks = constrain((long)(targetAngleDeg * (826.0f / 360.0f)), -STEER_MAX, STEER_MAX);
        targetTracTicks  = (long)(targetDistanceCm * TICKS_PER_CM);

        noInterrupts(); startTracPos = encTrac; interrupts();

        isArrivalActive = true;
        arrivalPhase    = 1;
        steeringStableCount = 0;
        steerPrevError  = 0.0f;
        fuzzyTrac.reset();
    }

    // === DIAGNOSTICS (raw PWM durante 3 segundos) ===
    else if (p.indexOf("\"diagnostics\"") != -1) {
        // Detener control normal
        applyTrac(0); applySteer(0);
        fuzzyTrac.reset(); steerPrevError = 0.0f;
        isArrivalActive = false; arrivalPhase = 0;
        targetSpeed = 0.0f;

        diagMotor    = (p.indexOf("\"steer\"") != -1) ? "steer" : "trac";
        diagPwm      = (int)parseVal(p, "\"pwm\"");
        diagPwm      = constrain(diagPwm, -255, 255);
        isDiagActive = true;
        diagStartTime = millis();
        Serial.printf("[DIAG] Motor=%s PWM=%d durante 3s\n", diagMotor.c_str(), diagPwm);
    }

    // === TUNE (sintonía en caliente de ganancias Fuzzy Tracción) ===
    else if (p.indexOf("\"tune\"") != -1) {
        float ge = parseVal(p, "\"ge\""), gde = parseVal(p, "\"gde\""), gu = parseVal(p, "\"gu\"");
        if (p.indexOf("\"motor\":\"trac\"") != -1) fuzzyTrac.setGains(ge, gde, gu);
        // No se sintoniza dirección por fuzzy — usa PD clásico
    }

    // === POLARITY (calibración de polaridades + swap) ===
    else if (p.indexOf("\"polarity\"") != -1) {
        int t_mot = (int)parseVal(p, "\"t_mot\"");
        int t_enc = (int)parseVal(p, "\"t_enc\"");
        int s_mot = (int)parseVal(p, "\"s_mot\"");
        int s_enc = (int)parseVal(p, "\"s_enc\"");
        int swap  = (int)parseVal(p, "\"swap\"");

        if (t_mot == 1 || t_mot == -1) tracMotorDir  = t_mot;
        if (t_enc == 1 || t_enc == -1) tracEncDir    = t_enc;
        if (s_mot == 1 || s_mot == -1) steerMotorDir = s_mot;
        if (s_enc == 1 || s_enc == -1) steerEncDir   = s_enc;
        swapHardware = (swap == 1);

        prefs.begin("polarity", false);
        prefs.putInt("trac_mot", (int)tracMotorDir);
        prefs.putInt("trac_enc", (int)tracEncDir);
        prefs.putInt("steer_mot", (int)steerMotorDir);
        prefs.putInt("steer_enc", (int)steerEncDir);
        prefs.putBool("swap_hw", swapHardware);
        prefs.end();

        Serial.printf("[POL] TM=%d TE=%d SM=%d SE=%d SWAP=%s\n",
                      tracMotorDir, tracEncDir, steerMotorDir, steerEncDir,
                      swapHardware ? "SI" : "NO");
    }

    // === RESET (reiniciar encoders y estado) ===
    else if (p.indexOf("\"reset\"") != -1) {
        fullStop();
        noInterrupts(); encTrac = 0; encSteer = 0; interrupts();
        lastEncTrac = 0;
        Serial.println("[RESET] Encoders y estado reiniciados.");
    }
}

// --------------------------------------------------
// WiFi
// --------------------------------------------------
void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("WiFi");
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        delay(500); Serial.print('.');
    }
    Serial.println(WiFi.status() == WL_CONNECTED ? " OK" : " FAIL");
}

// --------------------------------------------------
// MQTT Reconexión No Bloqueante
// --------------------------------------------------
void reconnectMQTT() {
    static unsigned long lastAttempt = 0;
    unsigned long now = millis();
    if (now - lastAttempt < 5000) return;
    lastAttempt = now;

    Serial.print("MQTT...");
    if (mqttClient.connect(clientId.c_str())) {
        mqttClient.subscribe(topicCommand.c_str());
        mqttClient.subscribe(topicConfig.c_str());
        Serial.println("OK");
    } else {
        Serial.print("ERR "); Serial.println(mqttClient.state());
    }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Cargar calibraciones de NVS
    prefs.begin("polarity", false);
    tracMotorDir  = prefs.getInt("trac_mot", 1);
    tracEncDir    = prefs.getInt("trac_enc", 1);
    steerMotorDir = prefs.getInt("steer_mot", 1);
    steerEncDir   = prefs.getInt("steer_enc", 1);
    swapHardware  = prefs.getBool("swap_hw", false);
    prefs.end();

    // Validar polaridades
    if (tracMotorDir  != 1 && tracMotorDir  != -1) tracMotorDir  = 1;
    if (tracEncDir    != 1 && tracEncDir    != -1) tracEncDir    = 1;
    if (steerMotorDir != 1 && steerMotorDir != -1) steerMotorDir = 1;
    if (steerEncDir   != 1 && steerEncDir   != -1) steerEncDir   = 1;

    // Pines de motor
    pinMode(PIN_TRAC_IN1,  OUTPUT); pinMode(PIN_TRAC_IN2,  OUTPUT);
    pinMode(PIN_STEER_IN1, OUTPUT); pinMode(PIN_STEER_IN2, OUTPUT);

    // PWM LEDC
    setupLEDC(PIN_PWM_TRAC,  PWM_FREQ, PWM_RES, PWM_CH_TRAC);
    setupLEDC(PIN_PWM_STEER, PWM_FREQ, PWM_RES, PWM_CH_STEER);
    applyTrac(0); applySteer(0);

    // Encoders
    pinMode(PIN_ENC_TRAC_A,  INPUT);
    pinMode(PIN_ENC_TRAC_B,  INPUT);
    pinMode(PIN_ENC_STEER_A, INPUT_PULLUP);
    pinMode(PIN_ENC_STEER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_TRAC_A),  isrTrac,  RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_STEER_A), isrSteer, RISING);

    // Topics MQTT únicos por MAC
    uint8_t mac[6]; WiFi.macAddress(mac);
    char ms[13];
    snprintf(ms, sizeof(ms), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    macStr         = String(ms);
    clientId       = "ESP32Ack_" + macStr;
    topicTelemetry = String(MQTT_BASE_TOPIC) + macStr + "/telemetry";
    topicCommand   = String(MQTT_BASE_TOPIC) + macStr + "/command";
    topicConfig    = String(MQTT_BASE_TOPIC) + macStr + "/config";

    Serial.println("\n==================================================");
    Serial.println("===   Fuzzy Robot ESP32 — Firmware v4.0       ===");
    Serial.printf ("MAC: %s\n", macStr.c_str());
    Serial.printf ("Swap hardware: %s\n", swapHardware ? "SI" : "NO");
    Serial.println("Motor TRAC (trasero)  → Control PI de VELOCIDAD");
    Serial.println("Motor STEER (delante) → Control PD de POSICIÓN");
    Serial.printf ("STEER_MAX: ±%ld ticks (±360°) — BLOQUEADO\n", STEER_MAX);
    Serial.println("==================================================\n");

    setupWiFi();
    mqttClient.setBufferSize(2048);
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    lastControlTime = lastTelemetryTime = millis();
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------
void loop() {

    // WiFi watchdog
    if (WiFi.status() != WL_CONNECTED) {
        static unsigned long wt = 0;
        if (millis() - wt > 10000) {
            wt = millis();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    } else {
        if (!mqttClient.connected()) {
            // Seguridad: si se pierde MQTT, detener motores
            applyTrac(0); applySteer(0);
            targetSpeed = 0.0f;
            reconnectMQTT();
        } else {
            mqttClient.loop();
        }
    }

    unsigned long now = millis();

    // ==========================================
    // DIAGNÓSTICO RAW (prioridad máxima)
    // ==========================================
    if (isDiagActive) {
        if (now - diagStartTime < DIAG_DURATION_MS) {
            // Aplicar PWM raw al motor solicitado
            if (diagMotor == "steer") {
                applyTrac(0);
                applySteer(diagPwm);
                currentPwmSteer = (float)diagPwm;
                currentPwmTrac  = 0.0f;
            } else {
                applySteer(0);
                applyTrac(diagPwm);
                currentPwmTrac  = (float)diagPwm;
                currentPwmSteer = 0.0f;
            }
        } else {
            // Tiempo de diagnóstico terminado
            isDiagActive = false;
            applyTrac(0); applySteer(0);
            currentPwmTrac  = 0.0f;
            currentPwmSteer = 0.0f;
            Serial.println("[DIAG] Diagnóstico terminado. Motores detenidos.");
        }
    }

    // ==========================================
    // LOOP DE CONTROL (20 Hz)
    // ==========================================
    if (!isDiagActive && (now - lastControlTime >= CONTROL_MS)) {
        float dt = (now - lastControlTime) / 1000.0f;
        lastControlTime = now;

        // Leer encoders de forma atómica
        noInterrupts();
        long tTrac  = encTrac;
        long tSteer = encSteer;
        interrupts();

        // Velocidad real de tracción (ticks/s)
        float speedTrac = (float)(tTrac - lastEncTrac) / dt;
        lastEncTrac = tTrac;

        if (isArrivalActive) {
            // ==========================================
            // MODO AUTOPILOT (llegada de precisión)
            // ==========================================
            if (arrivalPhase == 1) {
                // FASE 1: Orientar dirección — tracción frenada
                applyTrac(0);
                currentPwmTrac = 0.0f;

                // PD de dirección hacia targetAngleTicks
                currentPwmSteer = computeSteerPD(targetAngleTicks, tSteer, dt);
                applySteer((int)currentPwmSteer);

                // Verificar estabilización
                float steerErr = (float)(targetAngleTicks - tSteer);
                if (fabsf(steerErr) <= steerTolerance) {
                    steeringStableCount++;
                    if (steeringStableCount >= 4) { // 200ms estable
                        arrivalPhase = 2;
                        noInterrupts(); startTracPos = encTrac; interrupts();
                        fuzzyTrac.reset();
                        steeringStableCount = 0;
                        Serial.println("[AUTOPILOT] Fase 1 OK → Iniciando Fase 2 (traslación)");
                    }
                } else {
                    steeringStableCount = 0;
                }
            }
            else if (arrivalPhase == 2) {
                // FASE 2: Traslación — dirección bloqueada en posición
                currentPwmSteer = computeSteerPD(targetAngleTicks, tSteer, dt);
                applySteer((int)currentPwmSteer);

                long traveledTicks  = tTrac - startTracPos;
                long remainingTicks = targetTracTicks - traveledTicks;
                remainingDistCm = (float)remainingTicks / TICKS_PER_CM;

                // Perfil de velocidad: desaceleración proporcional
                float speedTarget = arrivalKpTrac * (float)remainingTicks;
                speedTarget = constrain(speedTarget, -maxArrivalSpeed, maxArrivalSpeed);
                if (fabsf(speedTarget) < 5.0f) speedTarget = 0.0f;

                currentPwmTrac = fuzzyTrac.compute(speedTarget, speedTrac, dt);
                applyTrac((int)currentPwmTrac);

                // Verificar llegada
                if (abs(remainingTicks) <= (long)arrivalTolerance && fabsf(speedTrac) < 5.0f) {
                    isArrivalActive = false;
                    arrivalPhase    = 0;
                    targetSpeed     = 0.0f;
                    fuzzyTrac.reset();
                    applyTrac(0); applySteer(0);
                    currentPwmTrac = 0.0f; currentPwmSteer = 0.0f;
                    remainingDistCm = 0.0f;
                    Serial.println("[AUTOPILOT] Destino alcanzado. Motores detenidos.");
                }
            }
        }
        else {
            // ==========================================
            // MODO MANUAL (teleoperación)
            // ==========================================

            // --- TRACCIÓN (motor trasero) → control de VELOCIDAD ---
            if (targetSpeed == 0.0f) {
                fuzzyTrac.reset();
                currentPwmTrac = 0.0f;
                applyTrac(0);
            } else {
                currentPwmTrac = fuzzyTrac.compute(targetSpeed, speedTrac, dt);
                applyTrac((int)currentPwmTrac);
            }

            // --- DIRECCIÓN (motor delantero) → control de POSICIÓN ---
            currentPwmSteer = computeSteerPD(targetAngle, tSteer, dt);
            applySteer((int)currentPwmSteer);
        }
    }

    // ==========================================
    // TELEMETRÍA (4 Hz)
    // ==========================================
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS && mqttClient.connected()) {
        lastTelemetryTime = now;

        noInterrupts();
        long posTrac  = encTrac;
        long posSteer = encSteer;
        interrupts();

        // Velocidad snapshot para telemetría
        static long lastPosTelTrac = 0;
        float spdSnap = (float)(posTrac - lastPosTelTrac) / (TELEMETRY_INTERVAL_MS / 1000.0f);
        lastPosTelTrac = posTrac;

        // Convertir posición de dirección a grados para la UI
        float steerTargetDeg = (float)(isArrivalActive ? targetAngleTicks : targetAngle) * (360.0f / 826.0f);
        float steerPosDeg    = (float)posSteer * (360.0f / 826.0f);
        float steerErrDeg    = steerTargetDeg - steerPosDeg;

        char buf[800];
        snprintf(buf, sizeof(buf),
            "{\"connected\":true,"
            "\"trac\":{\"target\":%.1f,\"speed\":%.1f,\"pos\":%ld,\"pwm\":%.1f},"
            "\"steer\":{\"target\":%.1f,\"pos\":%.1f,\"pwm\":%.1f,\"err\":%.1f},"
            "\"arrival\":{\"active\":%s,\"phase\":%d,\"target_dist\":%.1f,\"remaining_dist\":%.1f,\"target_angle\":%.1f},"
            "\"polarity\":{\"t_mot\":%d,\"t_enc\":%d,\"s_mot\":%d,\"s_enc\":%d,\"swap\":%d},"
            "\"diag\":{\"active\":%s,\"motor\":\"%s\",\"pwm\":%d},"
            "\"limits\":{\"steer_max\":%ld,\"steer_pos\":%ld,\"at_cw\":%s,\"at_ccw\":%s}"
            "}",
            targetSpeed, spdSnap, posTrac, currentPwmTrac,
            steerTargetDeg, steerPosDeg, currentPwmSteer, steerErrDeg,
            isArrivalActive ? "true" : "false", arrivalPhase, targetDistanceCm, remainingDistCm, targetAngleDeg,
            (int)tracMotorDir, (int)tracEncDir, (int)steerMotorDir, (int)steerEncDir, swapHardware ? 1 : 0,
            isDiagActive ? "true" : "false", diagMotor.c_str(), isDiagActive ? diagPwm : 0,
            STEER_MAX, posSteer,
            (posSteer >= STEER_MAX) ? "true" : "false",
            (posSteer <= -STEER_MAX) ? "true" : "false"
        );

        mqttClient.publish(topicTelemetry.c_str(), buf);
    }

    // Discovery ping cada 10 segundos
    static unsigned long lastDiscoveryTime = 0;
    if (now - lastDiscoveryTime >= 10000 && mqttClient.connected()) {
        lastDiscoveryTime = now;
        char discBuf[128];
        snprintf(discBuf, sizeof(discBuf),
                 "{\"mac\":\"%s\",\"status\":\"online\",\"ip\":\"%s\"}",
                 macStr.c_str(), WiFi.localIP().toString().c_str());
        mqttClient.publish("esp32/robot_fuzzy/discovery", discBuf);
    }
}
