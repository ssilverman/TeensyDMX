/*
 * Demonstrates DMX pixel output using FastLED.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2020-2021 Shawn Silverman
 */

#include <FastLED.h>

#include "CTeensyDMXLEDController.h"

static constexpr int kNumLEDs = 3;  // 3 pixels

CRGB leds[kNumLEDs];  // LED array

// TX on Serial3, start channel=1, RGB order, packet size=513
CTeensyDMXLEDController<> controller{Serial3};

void setup() {
  // Serial initialization, for printing things (optional)
  // Serial.begin(115200);
  // while (!Serial && millis() < 4000) {
  //   // Wait for Serial initialization
  // }
  // Serial.println("Starting.");

  // FastLED initialization; yours may be different
  FastLED.setBrightness(255);
  FastLED.addLeds(&controller, leds, kNumLEDs);  // How to use the controller
}

void loop() {
  // Blink RGB on three pixels
  // Your effects loop will be different

  leds[0] = CRGB::Red;
  leds[1] = CRGB::Green;
  leds[2] = CRGB::Blue;
  FastLED.show();
  delay(1000);

  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  FastLED.show();
  delay(1000);
}
