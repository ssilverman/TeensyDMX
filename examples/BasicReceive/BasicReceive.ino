/*
 * A basic toy receive example, demonstrating single- and
 * multi-byte reads.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2019-2020 Shawn Silverman
 */

#include <cstring>

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// The last value on the channel, for knowing when to print a change
// (Example 1).
uint8_t lastValue = 0;

// Buffer in which to store packet data (Example 2).
uint8_t packetBuf[3]{0};

// The last values received on channels 10-12, initialized to zero.
uint8_t rgb[3]{0};

void setup() {
  // Serial initialization, for printing things
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting BasicReceive.");

  // Turn on the LED, for indicating activity
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);

  // Start the receiver
  dmxRx.begin();

  // Print the first values
  lastValue = dmxRx.get(1);
  Serial.printf("Channel 1: %d\n", lastValue);

  // Note: If this reads < 3 bytes then the other values will stay at
  // zero (because 'rgb' was initialized to zero, above)
  dmxRx.readPacket(rgb, 10, 3);
  Serial.printf("RGB: %d %d %d\n", rgb[0], rgb[1], rgb[2]);
}

void loop() {
  // The following two examples print values when they change

  // Example 1. Get the current value of channel 1.
  // This will return zero for no data (and also for data that's zero)
  uint8_t v = dmxRx.get(1);
  if (v != lastValue) {
    lastValue = v;
    Serial.printf("Channel 1: %d\n", v);
  }

  // Example 2. Read channels 10-12.
  // A return of -1 means no data, and a value < 3 means that there
  // was data, but the received packet wasn't large enough to contain
  // channels 10-12
  int read = dmxRx.readPacket(packetBuf, 10, 3);
  if (read == 3) {
    if (memcmp(packetBuf, rgb, 3) != 0) {
      memcpy(rgb, packetBuf, 3);
      Serial.printf("RGB: %d %d %d\n", rgb[0], rgb[1], rgb[2]);
    }
  }
}
