// Tests the responder timing delay.

#include "TeensyDMX.h"
#include "RDMResponder.h"

// The pin on which to flash the LED.
constexpr uint8_t kLEDPin = LED_BUILTIN;

namespace teensydmx = ::qindesign::teensydmx;

// Create the DMX receiver on Serial1.
teensydmx::Receiver dmxRx{Serial1};

// Pin that enables RX or TX.
constexpr int kRXNotTXPin = 2;

// Prototyping manufacturer ID
constexpr uint8_t deviceUID[6] = {0x7f, 0xf0, 0x01, 0x02, 0x03, 0x04};
teensydmx::RDMResponder rdmResponder(deviceUID);

// Function that knows how to enable and disable RX and TX.
void setRXNotTX(bool flag) {
  digitalWriteFast(kRXNotTXPin, flag ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);  // USB serial is always 12Mbit/s
  while (!Serial && millis() < 3000) {}  // Wait for the serial monitor to come up
  Serial.println("Starting.");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(kRXNotTXPin, OUTPUT);
  setRXNotTX(true);

  Serial.print("Setting up and adding RDM responder...");
  dmxRx.addResponder(&rdmResponder);
  dmxRx.setSetRXNotTXFunc(setRXNotTX);
  Serial.println("done.");

  Serial.print("Starting DMX receiver...");
  dmxRx.begin();
  Serial.println("done.");
}

void loop() {
}
