/*
 * A basic toy send example.
 *
 * (c) 2019 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX sender on Serial1.
teensydmx::Sender dmxTx{Serial1};

// Data for 3 channels.
uint8_t data[3]{0x44, 0x88, 0xcc};

void setup() {
  // NOTE: Don't forget to set any pin that enables the transmitter

  dmxTx.begin();

  // Set some channel values. These are being set in setup() to illustrate that
  // values are 'sticky'. They stay set until changed. There's no special
  // function to call for each iteration of loop().

  // Set channel 1 to 128
  dmxTx.set(1, 128);

  // Set channels 10-12 to the 3 values in 'data'
  dmxTx.set(10, data, 3);
}

void loop() {
  // Do something, maybe alter channel values.
}
