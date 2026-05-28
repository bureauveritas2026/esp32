#ifndef CONFIG_H
#define CONFIG_H

// --- WiFi Configuration ---
// Replace with your local WiFi credentials
#define WIFI_SSID       "Bengy"
#define WIFI_PASSWORD   "gallito2813*"

// --- MQTT Broker Configuration ---
// We use a public, free HiveMQ broker. It requires no authentication.
#define MQTT_BROKER     "broker.hivemq.com"
#define MQTT_PORT       1883

// --- Base MQTT Topic ---
// The program will append the ESP32's MAC address to this base topic
// to form a completely unique topic, preventing collision with other users.
// e.g., "esp32/robot_fuzzy/A1B2C3"
#define MQTT_BASE_TOPIC "esp32/robot_fuzzy/"

// --- Telemetry settings ---
#define TELEMETRY_INTERVAL_MS 250 // Send updates 4 times per second (250ms) for smooth real-time web graphs

// --- Sign Direction Multipliers (Flipping polarity easily) ---
#define TRAC_MOTOR_DIR    1   // Set to 1 or -1 to flip traction motor direction
#define TRAC_ENC_DIR      1   // Set to 1 or -1 to flip traction encoder direction
#define STEER_MOTOR_DIR   1   // Set to 1 or -1 to flip steering motor direction
#define STEER_ENC_DIR     1   // Set to 1 or -1 to flip steering encoder direction

// --- Physical Constants for Precision Navigation ---
#define TICKS_PER_CM      25.0f   // 25 ticks = 1 cm (2500 ticks = 1 meter)
#define WHEELBASE_M       0.40f   // Wheelbase (40 cm) for Ackermann steering angle calculation

#endif // CONFIG_H
