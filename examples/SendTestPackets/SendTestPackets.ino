/*
 * Demonstrates outputting DMX test packets (start code 0x55).
 * See section D3 of the DMX specification.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2019-2020 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Pin for enabling or disabling the transmitter.
// This may not be needed for your hardware.
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

  // Set appropriate BREAK and MAB times:
  // Bit time=9us, format=8E1: BREAK=90us, MAB=9us
  // Note that the MAB time might be larger than expected;
  // if too large, try the timer method below
  dmxTx.setBreakSerialParams(1000000/9, SERIAL_8E1);

  // Optionally, we can try setting the times directly,
  // but they'll likely be larger by a few microseconds
  // dmxTx.setBreakTime(88);  // Should be in the range 88-120us
  // dmxTx.setMABTime(8);     // Should be in the range 8-16us
  // dmxTx.setBreakUseTimerNotSerial(true);

  dmxTx.begin();
  Serial.println("Started.");
}

void loop() {
  // Do stuff here
}
