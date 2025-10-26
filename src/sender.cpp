#include <LoRa.h>
#include <SPI.h>

// Pin definitions (use constexpr instead of macros)
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

static unsigned long messageCounter = 0;

void setup() {
    Serial.begin(115200);
    while (!Serial) { /* wait for serial connection on some boards */
    }
    Serial.println("LoRa Sender Starting...");

    LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);

    if (!LoRa.begin(LORA_FREQUENCY)) {
        Serial.println("Starting LoRa failed!");
        while (true) {
            delay(1000);
        }
    }

    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setCodingRate4(LORA_CODING_RATE4);
    LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.enableCrc();

    Serial.println("LoRa started successfully.");
}

void sendLoRaMessage(const char* message) {
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();
}

void loop() {
    ++messageCounter;
    const unsigned long timestamp = millis();
    const float sensorData = random(200, 300) / 10.0f;

    // Use a stack buffer and snprintf to avoid heavy String concatenation
    char payload[64];
    int n = snprintf(payload, sizeof(payload), "%lu|%lu|%.3f",
                     messageCounter, timestamp, sensorData);
    if (n <= 0 || static_cast<size_t>(n) >= sizeof(payload)) {
        Serial.println("Payload formatting error");
        delay(PACKET_INTERVAL_MS);
        return;
    }

    sendLoRaMessage(payload);
    Serial.print("Sent: ");
    Serial.println(payload);

    delay(PACKET_INTERVAL_MS);
}
