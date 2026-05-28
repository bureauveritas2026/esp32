#include <WiFi.h>
#include <PubSubClient.h>
#include "PinDefinitions.h"
#include "Config.h"
#include "FuzzyController.h"

// --- Global Objects ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Initialize Fuzzy Controllers for Left and Right wheels
// Default Gains: Ge = 0.04 (Error Gain), Gde = 0.08 (Change of Error Gain), Gu = 8.0 (Output Delta PWM Gain)
FuzzyController fuzzyLeft(0.04f, 0.08f, 8.0f);
FuzzyController fuzzyRight(0.04f, 0.08f, 8.0f);

// --- State Variables ---
volatile long encoderLeftTicks = 0;
volatile long encoderRightTicks = 0;

long lastEncoderLeftTicks = 0;
long lastEncoderRightTicks = 0;

float speedLeft = 0.0f;  // ticks per second
float speedRight = 0.0f; // ticks per second

float targetSpeedLeft = 0.0f;  // Target speed in ticks/sec
float targetSpeedRight = 0.0f; // Target speed in ticks/sec

float currentPwmLeft = 0.0f;
float currentPwmRight = 0.0f;

unsigned long lastControlTime = 0;
unsigned long lastTelemetryTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 50; // 20 Hz control loop

// Dynamic MQTT Topic Strings based on MAC Address
String clientId;
String topicTelemetry;
String topicCommand;
String topicConfig;

// --- Interrupt Service Routines (ISRs) for Encoders ---
// Reading Channel B on rising edge of Channel A determines direction
void IRAM_ATTR leftEncoderISR() {
    if (digitalRead(PIN_ENC_A_B) == HIGH) {
        encoderLeftTicks++;
    } else {
        encoderLeftTicks--;
    }
}

void IRAM_ATTR rightEncoderISR() {
    if (digitalRead(PIN_ENC_B_B) == HIGH) {
        encoderRightTicks++;
    } else {
        encoderRightTicks--;
    }
}

// --- Motor Control Helper ---
void setMotorSpeeds(int pwmLeft, int pwmRight) {
    // Left Motor (Motor A)
    if (pwmLeft > 0) {
        digitalWrite(PIN_AIN1, HIGH);
        digitalWrite(PIN_AIN2, LOW);
        ledcWrite(PWM_CH_LEFT, constrain(pwmLeft, 0, 255));
    } else if (pwmLeft < 0) {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, HIGH);
        ledcWrite(PWM_CH_LEFT, constrain(abs(pwmLeft), 0, 255));
    } else {
        digitalWrite(PIN_AIN1, LOW);
        digitalWrite(PIN_AIN2, LOW);
        ledcWrite(PWM_CH_LEFT, 0);
    }

    // Right Motor (Motor B)
    if (pwmRight > 0) {
        digitalWrite(PIN_BIN1, HIGH);
        digitalWrite(PIN_BIN2, LOW);
        ledcWrite(PWM_CH_RIGHT, constrain(pwmRight, 0, 255));
    } else if (pwmRight < 0) {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, HIGH);
        ledcWrite(PWM_CH_RIGHT, constrain(abs(pwmRight), 0, 255));
    } else {
        digitalWrite(PIN_BIN1, LOW);
        digitalWrite(PIN_BIN2, LOW);
        ledcWrite(PWM_CH_RIGHT, 0);
    }
}

// --- Dynamic Parser for JSON-like commands ---
// Avoids external library dependencies for simplicity
float parseJsonValue(String payload, String key) {
    int keyIdx = payload.indexOf("\"" + key + "\"");
    if (keyIdx == -1) return 0.0f;
    
    int colonIdx = payload.indexOf(":", keyIdx);
    if (colonIdx == -1) return 0.0f;
    
    // Find where the number ends (comma or closing brace)
    int commaIdx = payload.indexOf(",", colonIdx);
    int braceIdx = payload.indexOf("}", colonIdx);
    int endIdx = (commaIdx != -1 && commaIdx < braceIdx) ? commaIdx : braceIdx;
    
    if (endIdx == -1) return 0.0f;
    
    String valStr = payload.substring(colonIdx + 1, endIdx);
    valStr.trim();
    return valStr.toFloat();
}

// --- MQTT Incoming Messages Callback ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String payloadStr = "";
    for (unsigned int i = 0; i < length; i++) {
        payloadStr += (char)payload[i];
    }
    
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(payloadStr);

    if (payloadStr.indexOf("\"cmd\":\"stop\"") != -1) {
        targetSpeedLeft = 0.0f;
        targetSpeedRight = 0.0f;
        fuzzyLeft.reset();
        fuzzyRight.reset();
        setMotorSpeeds(0, 0);
        Serial.println("Robot STOPPED");
    } 
    else if (payloadStr.indexOf("\"cmd\":\"drive\"") != -1) {
        targetSpeedLeft = parseJsonValue(payloadStr, "left");
        targetSpeedRight = parseJsonValue(payloadStr, "right");
        Serial.print("Set targets: Left=");
        Serial.print(targetSpeedLeft);
        Serial.print(", Right=");
        Serial.println(targetSpeedRight);
    } 
    else if (payloadStr.indexOf("\"cmd\":\"tune\"") != -1) {
        float ge = parseJsonValue(payloadStr, "ge");
        float gde = parseJsonValue(payloadStr, "gde");
        float gu = parseJsonValue(payloadStr, "gu");
        
        fuzzyLeft.setGains(ge, gde, gu);
        fuzzyRight.setGains(ge, gde, gu);
        
        Serial.print("Fuzzy Gains Updated: Ge=");
        Serial.print(ge);
        Serial.print(", Gde=");
        Serial.print(gde);
        Serial.print(", Gu=");
        Serial.println(gu);
    }
    else if (payloadStr.indexOf("\"cmd\":\"reset\"") != -1) {
        noInterrupts();
        encoderLeftTicks = 0;
        encoderRightTicks = 0;
        interrupts();
        lastEncoderLeftTicks = 0;
        lastEncoderRightTicks = 0;
        fuzzyLeft.reset();
        fuzzyRight.reset();
        Serial.println("Encoders and Fuzzy Controllers Reset");
    }
}

// --- WiFi Connection ---
void setupWiFi() {
    delay(10);
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20) {
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("");
        Serial.println("WiFi connection failed! Will retry in main loop.");
    }
}

// --- MQTT Connection ---
void reconnectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("connected!");
            mqttClient.subscribe(topicCommand.c_str());
            mqttClient.subscribe(topicConfig.c_str());
            Serial.print("Subscribed to command topic: ");
            Serial.println(topicCommand);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

// --- Setup ---
void setup() {
    Serial.begin(115200);

    // 1. Initialize Pin Modes
    pinMode(PIN_STBY, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT);
    pinMode(PIN_BIN2, OUTPUT);

    digitalWrite(PIN_STBY, HIGH); // Enable motor driver

    // 2. Setup PWM channels
    ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_PWMA, PWM_CH_LEFT);
    ledcAttachPin(PIN_PWMB, PWM_CH_RIGHT);

    setMotorSpeeds(0, 0);

    // 3. Setup Encoder Pins (with internal pullups)
    pinMode(PIN_ENC_A_A, INPUT_PULLUP);
    pinMode(PIN_ENC_A_B, INPUT_PULLUP);
    pinMode(PIN_ENC_B_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B_A), rightEncoderISR, RISING);

    // 4. Generate unique client ID & topics based on MAC address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    char macStr[13];
    snprintf(macStr, sizeof(macStr), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    clientId = "ESP32Robot_" + String(macStr);
    topicTelemetry = String(MQTT_BASE_TOPIC) + String(macStr) + "/telemetry";
    topicCommand = String(MQTT_BASE_TOPIC) + String(macStr) + "/command";
    topicConfig = String(MQTT_BASE_TOPIC) + String(macStr) + "/config";

    Serial.println("=================================================");
    Serial.print("Device Client ID: ");
    Serial.println(clientId);
    Serial.print("Telemetry Topic:  ");
    Serial.println(topicTelemetry);
    Serial.print("Command Topic:    ");
    Serial.println(topicCommand);
    Serial.println("=================================================");

    // 5. Connect to WiFi and configure MQTT client
    setupWiFi();
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    lastControlTime = millis();
    lastTelemetryTime = millis();
}

// --- Loop ---
void loop() {
    // 1. Maintain WiFi and MQTT connection
    if (WiFi.status() != WL_CONNECTED) {
        // Simple non-blocking reconnection attempt if WiFi drops
        static unsigned long lastWiFiRetry = 0;
        if (millis() - lastWiFiRetry > 10000) {
            lastWiFiRetry = millis();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    } else {
        if (!mqttClient.connected()) {
            reconnectMQTT();
        }
        mqttClient.loop();
    }

    unsigned long currentTime = millis();

    // 2. Control Loop execution (20 Hz / 50ms interval)
    if (currentTime - lastControlTime >= CONTROL_INTERVAL_MS) {
        float dt = (currentTime - lastControlTime) / 1000.0f;
        lastControlTime = currentTime;

        // Atomically copy and reset encoder ticks to prevent interrupt interference
        noInterrupts();
        long currentLeftTicks = encoderLeftTicks;
        long currentRightTicks = encoderRightTicks;
        interrupts();

        // Calculate Speeds (Ticks / Sec)
        long deltaTicksLeft = currentLeftTicks - lastEncoderLeftTicks;
        long deltaTicksRight = currentRightTicks - lastEncoderRightTicks;

        speedLeft = (float)deltaTicksLeft / dt;
        speedRight = (float)deltaTicksRight / dt;

        lastEncoderLeftTicks = currentLeftTicks;
        lastEncoderRightTicks = currentRightTicks;

        // Run Fuzzy logic control computations
        currentPwmLeft = fuzzyLeft.compute(targetSpeedLeft, speedLeft, dt);
        currentPwmRight = fuzzyRight.compute(targetSpeedRight, speedRight, dt);

        // Apply outputs to Motor Driver
        setMotorSpeeds((int)currentPwmLeft, (int)currentPwmRight);
    }

    // 3. Telemetry Transmission (Interval in Config.h)
    if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryTime = currentTime;

        if (mqttClient.connected()) {
            // Buffer to hold telemetry payload
            char payload[1024];

            // Safely fetch latest counts
            noInterrupts();
            long finalLeftPos = encoderLeftTicks;
            long finalRightPos = encoderRightTicks;
            interrupts();

            // Format telemetry frame manually to avoid ArduinoJson dependency overhead
            snprintf(payload, sizeof(payload),
                "{\"connected\":true,"
                "\"left\":{\"target\":%.1f,\"speed\":%.1f,\"pos\":%ld,\"pwm\":%.1f,\"err\":%.2f,\"derr\":%.2f,\"du\":%.2f},"
                "\"right\":{\"target\":%.1f,\"speed\":%.1f,\"pos\":%ld,\"pwm\":%.1f,\"err\":%.2f,\"derr\":%.2f,\"du\":%.2f},"
                "\"fuzzy\":{"
                    "\"left_mu_e\":[%.2f,%.2f,%.2f,%.2f,%.2f],\"left_mu_de\":[%.2f,%.2f,%.2f],"
                    "\"right_mu_e\":[%.2f,%.2f,%.2f,%.2f,%.2f],\"right_mu_de\":[%.2f,%.2f,%.2f]"
                "}}",
                targetSpeedLeft, speedLeft, finalLeftPos, currentPwmLeft, 
                fuzzyLeft.lastState.error, fuzzyLeft.lastState.d_error, fuzzyLeft.lastState.delta_pwm,
                targetSpeedRight, speedRight, finalRightPos, currentPwmRight, 
                fuzzyRight.lastState.error, fuzzyRight.lastState.d_error, fuzzyRight.lastState.delta_pwm,
                
                fuzzyLeft.lastState.mu_e[0], fuzzyLeft.lastState.mu_e[1], fuzzyLeft.lastState.mu_e[2], fuzzyLeft.lastState.mu_e[3], fuzzyLeft.lastState.mu_e[4],
                fuzzyLeft.lastState.mu_de[0], fuzzyLeft.lastState.mu_de[1], fuzzyLeft.lastState.mu_de[2],
                
                fuzzyRight.lastState.mu_e[0], fuzzyRight.lastState.mu_e[1], fuzzyRight.lastState.mu_e[2], fuzzyRight.lastState.mu_e[3], fuzzyRight.lastState.mu_e[4],
                fuzzyRight.lastState.mu_de[0], fuzzyRight.lastState.mu_de[1], fuzzyRight.lastState.mu_de[2]
            );

            mqttClient.publish(topicTelemetry.c_str(), payload);
        }
    }
}
