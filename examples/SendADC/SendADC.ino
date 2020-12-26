/*
 * Sends the value from an ADC.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2020 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Pin for enabling or disabling the transmitter.
// This may not be needed for your hardware.
constexpr uint8_t kTXPin = 17;

// Analog input pin.
constexpr uint8_t kAnalogPin = A0;

// The DMX channel to set with the analog value.
constexpr int kChannel = 1;

// readAnalog() declaration.
void readAnalog();

// Create the DMX sender on Serial3.
teensydmx::Sender dmxTx{Serial3};

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // Turn on the LED, for indicating activity
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);

  // Set the pin that enables the transmitter; may not be needed
  pinMode(kTXPin, OUTPUT);
  digitalWriteFast(kTXPin, HIGH);

  // If you want, set the packet size and refresh rate here

  // Set an initial value
  readAnalog();

  dmxTx.begin();
}

void loop() {
  // The analog read takes approximately 10-15us
  readAnalog();

  // Delay because we only need to sample as quickly as the DMX
  // refresh rate. The rate depends on the packet size and whatever's
  // set as the refresh rate in the API. We want to delay something
  // not greater than the period.
  delay(22);  // Approximately the duration of a full packet
              // at maximum rate
}

// Performs an analog read and sets the DMX channel with the value.
void readAnalog() {
  int val = analogRead(kAnalogPin) >> 2;  // 10 bits -> 8 bits
  dmxTx.set(kChannel, val);
}
