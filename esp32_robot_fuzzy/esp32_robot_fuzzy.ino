#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_arduino_version.h>
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

// Traction: Fuzzy PI (velocity control, rear motor)
// Steering: Fuzzy PD (position control, front motor)
// Gains: Ge, Gde, Gu, integrate
FuzzyController fuzzyTrac(0.04f, 0.08f, 8.0f, true);
FuzzyController fuzzySteer(0.06f, 0.12f, 30.0f, false);

// --- Encoder State ---
volatile long encTrac  = 0;
volatile long encSteer = 0;

long lastEncTrac = 0;

// --- Control Targets ---
float targetSpeed  = 0.0f;  // ticks/sec for traction motor
long  targetAngle  = 0;     // encoder ticks for steering angle

// Steering software limits (prevents physical over-rotation)
const long STEER_MAX = 200;

// --- Output Values ---
float currentPwmTrac  = 0.0f;
float currentPwmSteer = 0.0f;

// --- Timing ---
unsigned long lastControlTime   = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long CONTROL_MS = 50; // 20 Hz

// --- MQTT Topics ---
String clientId, topicTelemetry, topicCommand, topicConfig;

// ---- ISRs ----
void IRAM_ATTR isrTrac() {
    encTrac = encTrac + (digitalRead(PIN_ENC_TRAC_B) == HIGH ? 1 : -1);
}
void IRAM_ATTR isrSteer() {
    encSteer = encSteer + (digitalRead(PIN_ENC_STEER_B) == HIGH ? 1 : -1);
}

// ---- Motor Write ----
void setTraction(int pwm) {
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
        targetSpeed = 0; targetAngle = 0;
        fuzzyTrac.reset(); fuzzySteer.reset();
        setTraction(0); setSteering(0);
    } else if (p.indexOf("\"drive\"") != -1) {
        targetSpeed = parseVal(p, "\"speed\"");
        targetAngle = (long)parseVal(p, "\"angle\"");
        targetAngle = constrain(targetAngle, -STEER_MAX, STEER_MAX);
    } else if (p.indexOf("\"tune\"") != -1) {
        float ge = parseVal(p, "\"ge\""), gde = parseVal(p, "\"gde\""), gu = parseVal(p, "\"gu\"");
        if (p.indexOf("\"motor\":\"trac\"") != -1)  fuzzyTrac.setGains(ge, gde, gu);
        if (p.indexOf("\"motor\":\"steer\"") != -1) fuzzySteer.setGains(ge, gde, gu);
    } else if (p.indexOf("\"reset\"") != -1) {
        noInterrupts(); encTrac = 0; encSteer = 0; interrupts();
        lastEncTrac = 0;
        fuzzyTrac.reset(); fuzzySteer.reset();
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

    pinMode(PIN_STBY,      OUTPUT); digitalWrite(PIN_STBY, HIGH);
    pinMode(PIN_TRAC_IN1,  OUTPUT); pinMode(PIN_TRAC_IN2,  OUTPUT);
    pinMode(PIN_STEER_IN1, OUTPUT); pinMode(PIN_STEER_IN2, OUTPUT);

    setupLEDC(PIN_PWM_TRAC,  PWM_FREQ, PWM_RES, PWM_CH_TRAC);
    setupLEDC(PIN_PWM_STEER, PWM_FREQ, PWM_RES, PWM_CH_STEER);
    setTraction(0); setSteering(0);

    pinMode(PIN_ENC_TRAC_A,  INPUT_PULLUP); pinMode(PIN_ENC_TRAC_B,  INPUT_PULLUP);
    pinMode(PIN_ENC_STEER_A, INPUT_PULLUP); pinMode(PIN_ENC_STEER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_TRAC_A),  isrTrac,  RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_STEER_A), isrSteer, RISING);

    uint8_t mac[6]; WiFi.macAddress(mac);
    char ms[13];
    snprintf(ms, sizeof(ms), "%02X%02X%02X%02X%02X%02X", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    clientId      = "ESP32Ack_" + String(ms);
    topicTelemetry = String(MQTT_BASE_TOPIC) + ms + "/telemetry";
    topicCommand   = String(MQTT_BASE_TOPIC) + ms + "/command";
    topicConfig    = String(MQTT_BASE_TOPIC) + ms + "/config";

    Serial.println("=== Ackermann Robot ===");
    Serial.println("Topic: " + topicTelemetry);

    setupWiFi();
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
        currentPwmTrac = fuzzyTrac.compute(targetSpeed, speedTrac, dt);
        setTraction((int)currentPwmTrac);

        // Steering: position (ticks)
        currentPwmSteer = fuzzySteer.compute((float)targetAngle, (float)tSteer, dt);
        setSteering((int)currentPwmSteer);
    }

    // --- Telemetry (configurable interval) ---
    if (now - lastTelemetryTime >= TELEMETRY_INTERVAL_MS && mqttClient.connected()) {
        lastTelemetryTime = now;

        noInterrupts();
        long posTrac = encTrac; long posSteer = encSteer;
        interrupts();

        // Compute traction speed for telemetry snapshot
        float spdSnap = (float)(posTrac - lastEncTrac + (posTrac - lastEncTrac)); // approx
        char buf[768];
        snprintf(buf, sizeof(buf),
            "{\"connected\":true,"
             "\"trac\":{\"target\":%.1f,\"speed\":%.1f,\"pos\":%ld,\"pwm\":%.1f,"
                       "\"err\":%.2f,\"derr\":%.2f,\"du\":%.2f},"
             "\"steer\":{\"target\":%ld,\"pos\":%ld,\"pwm\":%.1f,"
                        "\"err\":%.2f,\"derr\":%.2f,\"du\":%.2f},"
             "\"fuzzy\":{"
               "\"trac_mu_e\":[%.2f,%.2f,%.2f,%.2f,%.2f],"
               "\"trac_mu_de\":[%.2f,%.2f,%.2f],"
               "\"steer_mu_e\":[%.2f,%.2f,%.2f,%.2f,%.2f],"
               "\"steer_mu_de\":[%.2f,%.2f,%.2f]"
             "}}",
            targetSpeed,
            fuzzyTrac.lastState.error + targetSpeed, // measured ≈ target - error
            posTrac, currentPwmTrac,
            fuzzyTrac.lastState.error, fuzzyTrac.lastState.d_error, fuzzyTrac.lastState.delta_pwm,

            targetAngle, posSteer, currentPwmSteer,
            fuzzySteer.lastState.error, fuzzySteer.lastState.d_error, fuzzySteer.lastState.delta_pwm,

            fuzzyTrac.lastState.mu_e[0],  fuzzyTrac.lastState.mu_e[1],
            fuzzyTrac.lastState.mu_e[2],  fuzzyTrac.lastState.mu_e[3],  fuzzyTrac.lastState.mu_e[4],
            fuzzyTrac.lastState.mu_de[0], fuzzyTrac.lastState.mu_de[1], fuzzyTrac.lastState.mu_de[2],

            fuzzySteer.lastState.mu_e[0],  fuzzySteer.lastState.mu_e[1],
            fuzzySteer.lastState.mu_e[2],  fuzzySteer.lastState.mu_e[3], fuzzySteer.lastState.mu_e[4],
            fuzzySteer.lastState.mu_de[0], fuzzySteer.lastState.mu_de[1],fuzzySteer.lastState.mu_de[2]
        );
        mqttClient.publish(topicTelemetry.c_str(), buf);
    }
}
