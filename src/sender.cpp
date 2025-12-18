#include <LoRa.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_adc_cal.h>

#include "HX711.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "soc/rtc.h"

static esp_adc_cal_characteristics_t adc_chars;
constexpr adc1_channel_t ADC_CHANNEL = ADC1_CHANNEL_4;  // GPIO32 = ADC1_CH4

// Battery measurement pins and constants
constexpr int PIN_GATE = 27;    // GPIO controlling MOSFET gate (BS170/IRLZ44N)
constexpr int PIN_ADC = 32;     // ESP32 ADC pin (ADC1_CH4)
constexpr float R1 = 300000.0;  // 300k ohm
constexpr float R2 = 120000.0;  // 120k ohm
constexpr float DIVIDER_RATIO = (R1 + R2) / R2;
constexpr int NUM_SAMPLES = 8;
constexpr float BATTERY_CALIBRATION_FACTOR = 0.979;  // Based on internal pin resistance

// LoRa pins
constexpr uint8_t DBG_LED_PIN = 2;
constexpr uint8_t PIN_SS = 5;     // SX1276 NSS (CS)
constexpr uint8_t PIN_RST = 17;   // SX1276 RESET
constexpr uint8_t PIN_DIO0 = 15;  // SX1276 DIO0 (interrupt)

// HX711 Load Cell pins and calibration factor
constexpr int LOADCELL_DOUT_PIN = 16;
constexpr int LOADCELL_SCK_PIN = 4;
constexpr float WEIGHT_CALIBRATION_FACTOR = 1069.518;
constexpr int SCALE_NUM_READINGS = 3;

// LoRa transmission constants
constexpr long LORA_FREQUENCY = 868E6;     // Hz
constexpr int LORA_SPREADING_FACTOR = 10;  // 7..12
constexpr long LORA_BANDWIDTH = 125E3;     // Hz
constexpr int LORA_CODING_RATE4 = 8;       // 5..8 (represents 4/5..4/8 as 5..8)
constexpr long LORA_PREAMBLE_LENGTH = 12;  // symbols
constexpr uint8_t LORA_SYNC_WORD = 0x12;   // network sync word
constexpr int LORA_TX_POWER = 20;          // dBm

constexpr unsigned long PACKET_INTERVAL_MS = 60 * 1000;  // delay between packets

// RTC memory
RTC_DATA_ATTR static uint32_t rtcMessageCounter = 0;
RTC_DATA_ATTR static uint64_t rtcTimestampMs = 0;

#define DEBUG

#if defined(DEBUG)
#define DBG_BEGIN(baud) Serial.begin(baud)
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)

#define DBG_LED_INIT()                  \
    do {                                \
        pinMode(DBG_LED_PIN, OUTPUT);   \
        digitalWrite(DBG_LED_PIN, LOW); \
    } while (0)
#define DBG_LED_ON() digitalWrite(DBG_LED_PIN, HIGH)
#define DBG_LED_OFF() digitalWrite(DBG_LED_PIN, LOW)
#else
#define DBG_BEGIN(baud) (void)0
#define DBG_PRINT(...) (void)0
#define DBG_PRINTLN(...) (void)0
#define DBG_LED_INIT() (void)0
#define DBG_LED_ON() (void)0
#define DBG_LED_OFF() (void)0
#endif

HX711 scale;
Preferences prefs;

/**
 * Initialize hardware: set CPU frequency and disable WiFi/Bluetooth to save power
 */
void initializeHardware() {
    // Reduce CPU frequency to 80MHz to save power
    rtc_cpu_freq_config_t config;
    rtc_clk_cpu_freq_get_config(&config);
    rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_80M, &config);
    rtc_clk_cpu_freq_set_config_fast(&config);

    // Disable WiFi and Bluetooth to save power
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_bluedroid_disable();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
}

/**
 * Initialize and configure LoRa module
 * @return true if initialization successful, false otherwise
 */
bool initializeLoRa() {
    DBG_PRINTLN("LoRa Sender Starting...");
    LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        DBG_PRINTLN("Starting LoRa failed!");
        return false;
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE4);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc();

    DBG_PRINTLN("LoRa started successfully.");
    return true;
}

/**
 * Initialize HX711 load cell scale and restore tare offset from NVS
 */
void initializeScale() {
    DBG_PRINTLN("Initializing the scale");
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale(WEIGHT_CALIBRATION_FACTOR);

    prefs.begin("scale", false);

    // Restore tare if stored to avoid repeated taring on every boot
    if (prefs.isKey("tare_val")) {
        long savedTare = prefs.getLong("tare_val", 0L);
        scale.set_offset(savedTare);
        DBG_PRINT("Restored tare offset from NVS: ");
        DBG_PRINTLN(savedTare);
    } else {
        // Perform initial tare and store it
        scale.tare();
        long offset = scale.get_offset();
        prefs.putLong("tare_val", offset);
        DBG_PRINT("Performed initial tare and stored offset: ");
        DBG_PRINTLN(offset);
    }

    prefs.end();
}

/**
 * Initialize ADC for battery voltage measurement
 */
void initializeBatteryADC() {
    // Configure MOSFET gate pin
    pinMode(PIN_GATE, OUTPUT);
    digitalWrite(PIN_GATE, LOW);

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_2_5);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, &adc_chars);
}

float readBatteryVoltage() {
    // Turn on MOSFET to enable battery voltage measurement
    digitalWrite(PIN_GATE, HIGH);
    delayMicroseconds(200);  // Wait for voltage to stabilize

    long adcSum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adcSum += adc1_get_raw(ADC_CHANNEL);
        delayMicroseconds(100);  // Small delay between samples for stability
    }
    // Turn off MOSFET to save power
    digitalWrite(PIN_GATE, LOW);

    float adcAvg = adcSum / (float)NUM_SAMPLES;
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adcAvg, &adc_chars);
    float vAdc = voltage_mv / 1000.0;  // Convert mV to V
    float vBattery = vAdc * DIVIDER_RATIO * BATTERY_CALIBRATION_FACTOR;
    return vBattery;
}

/*
 * Calculate battery percentage from voltage using a 5th degree polynomial approximation
 * @param voltage Battery voltage in volts
 * @return Estimated battery percentage (0.0 to 100.0)
 */
float getBatteryPercentage(float voltage) {
    float result = 0.0;
    result += -21.407274 * pow(voltage, 5);
    result += 321.139954 * pow(voltage, 4);
    result += -1895.671518 * pow(voltage, 3);
    result += 5543.881032 * pow(voltage, 2);
    result += -8062.302892 * voltage;
    result += 4672.401703;
    if (result > 100.0) return 100.0;
    if (result < 0.0) return 0.0;
    return result;
}

/*
 * Calculate battery percentage from voltage using lookup table and linear interpolation
 * @param voltage Battery voltage in volts
 * @return Estimated battery percentage (0.0 to 100.0)
 */
float getBatteryPercentageLookup(float voltage) {
    const int numPoints = 20;
    float voltages[20] = {4.11, 4.02, 3.94, 3.85, 3.77, 3.68, 3.60, 3.51, 3.43, 3.34, 3.26, 3.17, 3.09, 3.00, 2.92, 2.83, 2.75, 2.66, 2.58, 2.49};
    float percentages[20] = {100.0, 95.5, 86.2, 76.2, 66.1, 56.3, 47.1, 38.7, 31.2, 24.6, 19.1, 14.5, 10.7, 7.7, 5.4, 3.5, 2.2, 1.1, 0.4, 0.0};
    // Linear interpolation
    if (voltage >= voltages[0]) return percentages[0];
    if (voltage <= voltages[numPoints - 1]) return percentages[numPoints - 1];
    for (int i = 0; i < numPoints - 1; i++) {
        if (voltage <= voltages[i] && voltage >= voltages[i + 1]) {
            float t = (voltage - voltages[i + 1]) / (voltages[i] - voltages[i + 1]);
            return percentages[i + 1] + t * (percentages[i] - percentages[i + 1]);
        }
    }
    return 0.0;
}

/**
 * Build the LoRa payload string
 * @param buffer Output buffer for the payload
 * @param bufferSize Size of the output buffer
 * @param msgCount Message counter
 * @param timestamp Timestamp in milliseconds
 * @param weight Weight reading in grams
 * @param voltage Battery voltage in volts
 * @param batteryPct Battery percentage
 * @return true if payload was built successfully, false on error
 */
bool buildPayload(char* buffer, size_t bufferSize,
                  uint32_t msgCount, unsigned long timestamp,
                  float weight, float voltage, float batteryPct) {
    // Format: messageCounter|timestampMs|weight|batteryVoltage|batteryPercentage
    int bytesWritten = snprintf(buffer, bufferSize, "%lu|%lu|%.3f|%.3f|%.2f",
                                msgCount, timestamp, weight, voltage, batteryPct);

    if (bytesWritten <= 0 || static_cast<size_t>(bytesWritten) >= bufferSize) {
        DBG_PRINTLN("Payload formatting error");
        return false;
    }

    return true;
}

/**
 * Send a LoRa packet with the given payload
 * @param payload Null-terminated string to send
 */
void sendLoRaPacket(const char* payload) {
    const unsigned long beforeSendMillis = millis();

    DBG_PRINTLN("Sending LoRa packet...");
    DBG_LED_ON();

    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();

    DBG_LED_OFF();
    DBG_PRINT("Sent: ");
    DBG_PRINTLN(payload);
    DBG_PRINT("LoRa Tx time (ms): ");
    DBG_PRINTLN(millis() - beforeSendMillis);

    LoRa.sleep();
}

/**
 * Enter deep sleep mode for the specified interval
 * @param intervalMs Sleep duration in milliseconds
 */
void enterDeepSleep(unsigned long intervalMs) {
    DBG_PRINTLN("Going to deep sleep...");
    // Configure deep sleep timer (milliseconds -> microseconds)
    esp_sleep_enable_timer_wakeup(intervalMs * 1000ULL);
    esp_deep_sleep_start();
}

void setup() {
    DBG_BEGIN(115200);
    unsigned long sessionStartMs = millis();

    DBG_PRINT("Current RTC Timestamp (ms): ");
    DBG_PRINTLN(rtcTimestampMs + sessionStartMs);

    DBG_LED_INIT();

    // Initialize all hardware subsystems
    initializeHardware();

    if (!initializeLoRa()) {
        // LoRa failed to start, sleep and retry later
        enterDeepSleep(PACKET_INTERVAL_MS);
    }

    initializeScale();
    initializeBatteryADC();

    // Read battery status
    float batteryVoltage = readBatteryVoltage();
    float batteryPercentage = getBatteryPercentage(batteryVoltage);
    DBG_PRINT("Battery Voltage (V): ");
    DBG_PRINTLN(batteryVoltage);
    DBG_PRINT("BatteryPercent: ");
    DBG_PRINT(batteryPercentage, 1);
    DBG_PRINTLN("%");

    // Read weight from scale
    DBG_LED_ON();
    float weight = scale.get_units(SCALE_NUM_READINGS);
    DBG_PRINT("Weight reading (g): ");
    DBG_PRINTLN(weight);
    DBG_LED_OFF();

    DBG_PRINT("Weigh time (ms): ");
    DBG_PRINTLN(millis() - sessionStartMs);
    scale.power_down();

    // Prepare data for transmission
    ++rtcMessageCounter;
    const unsigned long currentTimestamp = static_cast<unsigned long>(rtcTimestampMs + millis());

    // Build and send payload
    char payload[64];
    if (!buildPayload(payload, sizeof(payload),
                      rtcMessageCounter, currentTimestamp,
                      weight, batteryVoltage, batteryPercentage)) {
        // Payload formatting failed, sleep and retry
        enterDeepSleep(PACKET_INTERVAL_MS);
    }

    sendLoRaPacket(payload);

    // Update timestamp and enter deep sleep
    rtcTimestampMs += PACKET_INTERVAL_MS + millis();
    enterDeepSleep(PACKET_INTERVAL_MS);
}

void loop() {
    // Not used in deep sleep mode
}
