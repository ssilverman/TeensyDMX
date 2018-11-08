// Test program that sends a basic RDM request every so often.

#include <Arduino.h>

// 50000 baud, 8N1: 180us break, 20us MAB
constexpr uint32_t kBreakBaud = 50000;
constexpr uint32_t kBreakFormat = SERIAL_8N1;
constexpr uint32_t kSlotsBaud = 250000;
constexpr uint32_t kSlotsFormat = SERIAL_8N2;

constexpr int kSendInterval = 2000;

constexpr uint8_t destUID[6] = {0x7f, 0xf0, 0x01, 0x02, 0x03, 0x04};
constexpr uint8_t srcUID[6] = {0x7f, 0xf0, 0x00, 0x00, 0x00, 0x01};

uint8_t kRDMPacket[26] = {
  0xcc,        // RDM start code
  0x01,        // Sub message
  24,          // Message length
  destUID[0], destUID[1], destUID[2], destUID[3], destUID[4], destUID[5],
  srcUID[0], srcUID[1], srcUID[2], srcUID[3], srcUID[4], srcUID[5],
  0,           // Transaction number
  1,           // Port ID
  0,           // Message count
  0, 0,        // Sub-device
  0x20,        // Command class: GET_COMMAND
  0x00, 0x50,  // Parameter ID: SUPPORTED_PARAMETERS
  0,           // Parameter data length
  0x00, 0x00,  // Checksum
};

HardwareSerial &serial{Serial1};
elapsedMillis timer{kSendInterval};

void setup() {
  Serial.begin(115200);  // USB serial is always 12Mbit/s
  while (!Serial && millis() < 3000) {
  }
  Serial.println("Starting.");

  // Calculate the packet checksum
  uint16_t checksum = 0;
  for (int i = 0; i < int{sizeof(kRDMPacket)} - 2; i++) {
    checksum += kRDMPacket[i];
  }
  kRDMPacket[24] = checksum >> 8;
  kRDMPacket[25] = checksum;

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (timer < kSendInterval) {
    return;
  }
  timer = 0;

  digitalWrite(LED_BUILTIN, HIGH);
  serial.begin(kBreakBaud, kBreakFormat);
  serial.write(0);
  serial.flush();
  serial.begin(kSlotsBaud, kSlotsFormat);
  serial.write(kRDMPacket, sizeof(kRDMPacket));
  serial.flush();
  digitalWrite(LED_BUILTIN, LOW);
}
