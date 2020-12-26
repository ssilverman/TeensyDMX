/*
 * A basic toy send example.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2019-2020 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Pin for enabling or disabling the transmitter.
// This may not be needed for your hardware.
constexpr uint8_t kTXPin = 17;

// Create the DMX sender on Serial1.
teensydmx::Sender dmxTx{Serial1};

// Data for 3 channels.
uint8_t data[3]{0x44, 0x88, 0xcc};

void setup() {
  // Turn on the LED, for indicating activity
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, HIGH);

  // Set the pin that enables the transmitter; may not be needed
  pinMode(kTXPin, OUTPUT);
  digitalWriteFast(kTXPin, HIGH);

  // Set some channel values. These are being set in setup() to
  // illustrate that values are 'sticky'. They stay set until changed.
  // There's no special function to call for each iteration of loop().

  // Set channel 1 to 128
  dmxTx.set(1, 128);

  // Set channels 10-12 to the 3 values in 'data'
  dmxTx.set(10, data, 3);

  // Call this after setting up the channel contents if you want to
  // guarantee that the values are used as the initial contents;
  // transmission doesn't start until after begin() is called.
  // If it doesn't matter, begin() can be called before setting the
  // channel contents.
  dmxTx.begin();
}

void loop() {
  // Do something, maybe alter channel values.
}
