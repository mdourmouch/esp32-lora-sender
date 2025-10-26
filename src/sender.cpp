#include <LoRa.h>
#include <SPI.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_sleep.h"
#include "esp_wifi.h"

// Pin definitions
constexpr uint8_t PIN_SS = 5;     // SX1276 NSS (CS)
constexpr uint8_t PIN_RST = 4;    // SX1276 RESET
constexpr uint8_t PIN_DIO0 = 26;  // SX1276 DIO0 (interrupt)

// LoRa configuration constants
constexpr long LORA_FREQUENCY = 868E6;     // Hz
constexpr int LORA_SPREADING_FACTOR = 10;  // 7..12
constexpr long LORA_BANDWIDTH = 125E3;     // Hz
constexpr int LORA_CODING_RATE4 = 8;       // 5..8 (represents 4/5..4/8 as 5..8)
constexpr long LORA_PREAMBLE_LENGTH = 12;  // symbols
constexpr uint8_t LORA_SYNC_WORD = 0x12;   // network sync word
constexpr int LORA_TX_POWER = 2;           // dBm

constexpr unsigned long PACKET_INTERVAL_MS = 5000;  // delay between packets

// RTC memory
RTC_DATA_ATTR static uint32_t rtcMessageCounter = 0;
RTC_DATA_ATTR static uint64_t rtcTimestampMs = 0;

#define DEBUG

#if defined(DEBUG)
#define DBG_BEGIN(baud) Serial.begin(baud)
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)

constexpr uint8_t DBG_LED_PIN = 2;

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

void setup() {
    DBG_BEGIN(115200);
    sessionStartMs = millis();

    DBG_PRINT("Current RTC Timestamp (ms): ");
    DBG_PRINTLN(rtcTimestampMs + sessionStartMs);

    DBG_LED_INIT();
    DBG_PRINTLN("LoRa Sender Starting...");

    LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);
    // Disable WiFi and Bluetooth to save power
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_bluedroid_disable();
    esp_bt_controller_disable();

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
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc();

    DBG_PRINTLN("LoRa started successfully.");

    ++rtcMessageCounter;

    const unsigned long timestampBeforeSend = static_cast<unsigned long>(rtcTimestampMs + millis());
    const float sensorData = random(200, 300) / 10.0f;

    // Build payload
    char payload[64];
    int n = snprintf(payload, sizeof(payload), "%lu|%lu|%.3f",
                     rtcMessageCounter, timestampBeforeSend, sensorData);
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(payload)) {
        DBG_PRINTLN("Payload formatting error");
        // go to sleep and retry
        esp_sleep_enable_timer_wakeup(PACKET_INTERVAL_MS * 1000ULL);
        esp_deep_sleep_start();
    }

    // Send payload
    DBG_LED_ON();
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();
    DBG_LED_OFF();
    DBG_PRINT("Sent: ");
    DBG_PRINTLN(payload);
    DBG_PRINT("LoRa Tx time (ms): ");
    DBG_PRINTLN(millis() - sessionStartMs);
    LoRa.sleep();

    rtcTimestampMs += PACKET_INTERVAL_MS + millis();

    // Configure deep sleep for the requested interval (milliseconds -> microseconds)
    esp_sleep_enable_timer_wakeup(PACKET_INTERVAL_MS * 1000ULL);
    DBG_PRINTLN("Going to deep sleep...");
    esp_deep_sleep_start();
}

void loop() {
    // not used: device sleeps between transmissions
}
