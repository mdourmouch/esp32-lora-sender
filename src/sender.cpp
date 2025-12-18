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
const adc1_channel_t ADC_CHANNEL = ADC1_CHANNEL_4;  // GPIO32 = ADC1_CH4

// Battery measurement pins and constants
const int PIN_GATE = 27;    // GPIO controlling MOSFET gate (BS170/IRLZ44N)
const int PIN_ADC = 32;     // ESP32 ADC pin (ADC1_CH4)
const float R1 = 300000.0;  // 300k ohm
const float R2 = 120000.0;  // 120k ohm
const float DIVIDER_RATIO = (R1 + R2) / R2;
const int NUM_SAMPLES = 8;
const float BATTERY_CALIBRATION_FACTOR = 0.979;

// LoRa pins
constexpr uint8_t DBG_LED_PIN = 2;
constexpr uint8_t PIN_SS = 5;     // SX1276 NSS (CS)
constexpr uint8_t PIN_RST = 17;   // SX1276 RESET
constexpr uint8_t PIN_DIO0 = 15;  // SX1276 DIO0 (interrupt)

// HX711 Load Cell pins and calibration factor
constexpr int LOADCELL_DOUT_PIN = 16;
constexpr int LOADCELL_SCK_PIN = 4;
constexpr float WEIGHT_CALIBRATION_FACTOR = 1069.518;

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
RTC_DATA_ATTR static uint32_t txPower = 20;

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

unsigned long sessionStartMs = 0;

HX711 scale;
Preferences prefs;

float readBatteryVoltage() {
    digitalWrite(PIN_GATE, HIGH);
    delayMicroseconds(200);

    long adcSum = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        adcSum += adc1_get_raw(ADC_CHANNEL);
        delayMicroseconds(100);
    }
    // Turn off MOSFET to save power
    digitalWrite(PIN_GATE, LOW);

    float adcAvg = adcSum / (float)NUM_SAMPLES;
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adcAvg, &adc_chars);
    float vAdc = voltage_mv / 1000.0;  // Convert mV to V
    float vBattery = vAdc * DIVIDER_RATIO * BATTERY_CALIBRATION_FACTOR;
    return vBattery;
}

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

void setup() {
    DBG_BEGIN(115200);
    sessionStartMs = millis();

    DBG_PRINT("Current RTC Timestamp (ms): ");
    DBG_PRINTLN(rtcTimestampMs + sessionStartMs);

    rtc_cpu_freq_config_t config;
    rtc_clk_cpu_freq_get_config(&config);
    rtc_clk_cpu_freq_to_config(RTC_CPU_FREQ_80M, &config);
    rtc_clk_cpu_freq_set_config_fast(&config);

    DBG_LED_INIT();
    DBG_PRINTLN("LoRa Sender Starting...");

    LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
    // Disable WiFi and Bluetooth to save power
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_bluedroid_disable();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        DBG_PRINTLN("Starting LoRa failed!");
        esp_sleep_enable_timer_wakeup(PACKET_INTERVAL_MS * 1000ULL);
        esp_deep_sleep_start();
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE4);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(txPower);
    LoRa.enableCrc();

    DBG_PRINTLN("LoRa started successfully.");

    DBG_PRINTLN("Initializing the scale");
    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale.set_scale(WEIGHT_CALIBRATION_FACTOR);

    prefs.begin("scale", false);
    // restore tare if stored to avoid repeated taring on every boot
    if (prefs.isKey("tare_val")) {
        long savedTare = prefs.getLong("tare_val", 0L);
        scale.set_offset(savedTare);
        DBG_PRINT("Restored tare offset from NVS: ");
        DBG_PRINTLN(savedTare);
    } else {
        // perform initial tare and store it
        scale.tare();
        long offs = scale.get_offset();
        prefs.putLong("tare_val", offs);
        DBG_PRINT("Performed initial tare and stored offset: ");
        DBG_PRINTLN(offs);
    }
    prefs.end();

    // Prepare ADC for battery voltage measurement
    pinMode(PIN_GATE, OUTPUT);
    digitalWrite(PIN_GATE, LOW);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_2_5);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, &adc_chars);

    float batteryVoltage = readBatteryVoltage();
    float batteryPercentage = getBatteryPercentage(batteryVoltage);
    DBG_PRINT("Battery Voltage (V): ");
    DBG_PRINTLN(batteryVoltage);

    DBG_PRINT("BatteryPercent: ");
    DBG_PRINT(batteryPercentage, 1);
    DBG_PRINTLN("%");

    DBG_LED_ON();
    float weight = scale.get_units(3);

    DBG_PRINT("Weight reading (g): ");
    DBG_PRINTLN(weight);
    DBG_LED_OFF();

    DBG_PRINT("Weigh time (ms): ");
    DBG_PRINTLN(millis() - sessionStartMs);
    scale.power_down();

    ++rtcMessageCounter;

    const unsigned long beforeSendMillis = millis();
    const unsigned long timestampBeforeSend = static_cast<unsigned long>(rtcTimestampMs + beforeSendMillis);

    // Build payload: messageCounter|timestampMs|weight|batteryVoltage|batteryPercentage
    char payload[64];
    int n = snprintf(payload, sizeof(payload), "%lu|%lu|%.3f|%.3f|%.2f",
                     rtcMessageCounter, timestampBeforeSend, weight, batteryVoltage, batteryPercentage);
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(payload)) {
        DBG_PRINTLN("Payload formatting error");
        // go to sleep and retry
        esp_sleep_enable_timer_wakeup(PACKET_INTERVAL_MS * 1000ULL);
        esp_deep_sleep_start();
    }

    // Send payload
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
    DBG_PRINT("txPower: ");
    DBG_PRINTLN(txPower);
    LoRa.sleep();

    rtcTimestampMs += PACKET_INTERVAL_MS + millis();

    DBG_PRINTLN("Going to deep sleep...");
    // Configure deep sleep for the requested interval (milliseconds -> microseconds)
    esp_sleep_enable_timer_wakeup(PACKET_INTERVAL_MS * 1000ULL);
    esp_deep_sleep_start();
}

void loop() {
}
