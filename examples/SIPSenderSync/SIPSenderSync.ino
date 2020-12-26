/*
 * Demonstration program that sends SIP checksum data for each regular
 * packet. This is meant as a rudimentary example of SIP and does not
 * use all the features of the packet type.
 *
 * This uses the polling approach with isTransmitting() to determine
 * when to send the next packet.
 *
 * This example is part of the TeensyDMX library.
 * (c) 2018-2020 Shawn Silverman
 */

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Pin for enabling or disabling the transmitter.
// This may not be needed for your hardware.
constexpr uint8_t kTXPin = 17;

// Create the DMX transmitter on Serial1.
teensydmx::Sender dmxTx{Serial1};

// The regular packet data. Use this to fill in the DMX packet and to
// calculate SIP data. To make it easy to set the packet data in the
// transmitter, and because packets with other start codes are being
// used, include the NULL start code here.
uint8_t packetData[513]{0};

// Holds the SIP data to send in a packet, including the start code.
uint8_t sipData[25]{0};

// SIP state
uint8_t sipSeqNum = 0; // Free running
bool firstSIPSent = false;  // Keep track of whether there's been at
                            // least one SIP packet, and if there has,
                            // count packets since then
uint16_t packetsSinceLastSIP = 0;

// Program setup.
void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting SIPSenderSync.");

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, HIGH);

  // Set the pin that enables the transmitter; may not be needed
  pinMode(kTXPin, OUTPUT);
  digitalWriteFast(kTXPin, HIGH);

  // Initialize the SIP data
  sipData[0] = 0xcf;  // SIP start code
  sipData[1] = 24;    // Packet size including checksum
  sipData[6] = 1;     // DMX universe

  // Pause first so that we send no packets when the sender starts
  dmxTx.pause();

  dmxTx.begin();
}

// Main program loop.
void loop() {
  digitalWriteFast(kLEDPin, HIGH);

  // Fill the regular packet data and wait until it's done
  fillRegularData();
  dmxTx.resumeFor(1);
  while (dmxTx.isTransmitting()) {
    yield();
  }

  digitalWriteFast(kLEDPin, LOW);

  // Fill the SIP data and wait until it's done
  fillSIPData();
  firstSIPSent = true;
  packetsSinceLastSIP = 0;  // Reset this count
  dmxTx.resumeFor(1);
  while (dmxTx.isTransmitting()) {
    yield();
  }
}

// Fills the current packet with desired data.
void fillRegularData() {
  // Set other dynamic data here too
  packetData[1] = 128;
  dmxTx.set(0, packetData, sizeof(packetData));
  if (firstSIPSent) {
    packetsSinceLastSIP++;
  }
}

// Fills the current packet with the SIP data. This assumes that
// the current packet data is in the 'packetData' variable.
void fillSIPData() {
  // Calculate data checksum
  uint16_t checksum = 0;
  for (uint8_t b : packetData) {
    checksum += b;
  }
  checksum = ~checksum;
  sipData[3] = checksum >> 8;
  sipData[4] = checksum;

  // SIP sequence number
  sipData[5] = sipSeqNum++;

  // Packets since last SIP
  sipData[11] = packetsSinceLastSIP >> 8;
  sipData[12] = packetsSinceLastSIP;

  // Calculate SIP packet checksum
  checksum = 0;
  for (int i = 0; i < 24; i++) {
    checksum += sipData[i];
  }
  sipData[24] = ~checksum;

  // sipData includes the SIP start code
  dmxTx.set(0, sipData, 25);
}
