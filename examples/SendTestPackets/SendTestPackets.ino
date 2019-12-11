/*
 * Demonstrates outputting DMX test packets (start code 0x55).
 *
 * (c) 2019 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Pin for enabling or disabling the transmitter. This may not be needed for
// your hardware.
constexpr uint8_t kTXPin = 17;

// Create the DMX transmitter on Serial3.
teensydmx::Sender dmxTx{Serial3};

// The current channel outputting a value.
int channel;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting TestPackets.");

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, LOW);

  // Set the pin that enables the transmitter; may not be needed
  pinMode(kTXPin, OUTPUT);
  digitalWriteFast(kTXPin, HIGH);

  Serial.println("Setting up test packet and starting TX...");

  // Set up the test packet, including the start code
  for (int i = 0; i < teensydmx::kMaxDMXPacketSize; i++) {
    dmxTx.set(i, 0x55);
  }

  // Set appropriate BREAK and MAB times
  // Set something at least a few microseconds less than the maximum allowed
  // MAB time because practically, the actual MAB time may be longer
  dmxTx.setBreakTime(108);  // Should be in the range 88-120us
  dmxTx.setMABTime(12);     // Should be in the range 8-16us

  dmxTx.begin();
  Serial.println("Started.");
}

void loop() {
  // Do stuff here
}
