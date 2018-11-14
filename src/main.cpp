// Basic main program, so it's easy to compile the project.

#include <Arduino.h>

// Project includes
#include "TeensyDMX.h"

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX transmitter on Serial1.
teensydmx::Sender dmxTx{Serial1};

int channel = 0;

void setup() {
  // Initialize the serial port and wait for initialization to complete
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
  }

  // Set up any pins
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);
  pinMode(2, OUTPUT);
  digitalWriteFast(2, HIGH);

  Serial.println("Hello, DMX World!");

  // dmxTx.setPacketSize(25);
  // dmxTx.setRefreshRate(40);
  dmxTx.begin();
  channel = 512;
}

void loop() {
  digitalWriteFast(LED_BUILTIN, HIGH);
  dmxTx.set(channel++, 0);
  if (channel > 512) {
    channel = 1;
  }
  dmxTx.set(channel, 255);
  delay(20);
  digitalWriteFast(LED_BUILTIN, LOW);

  delay(980);
}
