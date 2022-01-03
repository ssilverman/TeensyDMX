// Basic main program, so it's easy to compile the project.
//
// This uses the concept of sub-sketches, where serial input selects which
// sub-sketch to run. The initial running sketch is the Null sketch; it does
// nothing. The other sketches are:
// 1. Chaser: This cycles an output of 255 on each channel.
// 2. Flasher: Receives input and flashes an LED with a period proportional to
//    the input.
//
// (c) 2018-2021 Shawn Silverman

// Define MAIN_TEST_PROGRAM to use this test program.
#ifdef MAIN_TEST_PROGRAM

// C++ includes
#include <cstdint>
#include <limits>

#include <Arduino.h>
#include "TeensyDMX.h"

namespace teensydmx = ::qindesign::teensydmx;

// Basic sub-sketch. The constructor and destructor don't do setup or teardown.
class Sketch {
 public:
  Sketch() = default;
  virtual ~Sketch() = default;

  virtual void setup() {}
  virtual void tearDown() {}
  virtual void loop() {}

  // Sketch info
  virtual String name() = 0;
  virtual int type() = 0;
};

// NullSketch supports the Null Pattern.
class NullSketch final : public Sketch {
 public:
  NullSketch() = default;
  ~NullSketch() override = default;

  String name() override {
    return "Null";
  }

  void setup() override;

  int type() override {
    return 'n';
  }
};

// Chaser cycles an output of 255 on each channel.
class Chaser final : public Sketch {
 public:
  Chaser();
  ~Chaser() override = default;

  String name() override {
    return "Chaser";
  }

  void setup() override;
  void tearDown() override;
  void loop() override;

  int type() override {
    return 'c';
  }

 private:
  static constexpr int kChannel = 1;
  static constexpr unsigned long kChaseInterval = 1000;  // 1s
  static constexpr int kPacketSize = 513;
  static constexpr float kRefreshRate = std::numeric_limits<float>::infinity();

  teensydmx::Sender dmx_;
  int channel_;
  elapsedMillis lastChaseTimer_;
};

// Flasher flashes the LED according to a DMX input speed.
class Flasher final : public Sketch {
 public:
  Flasher();
  ~Flasher() override = default;

  String name() override {
    return "Flasher";
  }

  void setup() override;
  void tearDown() override;
  void loop() override;

  int type() override {
    return 'f';
  }

 private:
  static constexpr int kChannel = 1;
  static constexpr unsigned long kTimeout = 1000;

  // Minimum and maximum frequencies, expressed as a period in milliseconds
  static constexpr int32_t kPeriodMax = 1000;
  static constexpr int32_t kPeriodMin = 30;

  teensydmx::Receiver dmx_;
  uint8_t lastValue_;
  elapsedMillis lastFrameTimer_;

  // Wave values
  int32_t period_;
  int64_t phi_;
};

// The LED pin.
constexpr uint8_t kLEDPin = LED_BUILTIN;

// Pin for enabling or disabling the transmitter.
constexpr uint8_t kTXPin = 17;

// The serial port to use.
HardwareSerial &uart = Serial3;

// The current sketch
NullSketch nullSketch{};
Chaser chaser{};
Flasher flasher{};
Sketch *currSketch = &nullSketch;

// ---------------------------------------------------------------------------
//  Main program
// ---------------------------------------------------------------------------

// Changes the current sketch, if different. This ignores any unknown
// sketch type.
void changeSketch(int sketchType);

// Main program setup.
void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {
    // Wait for initialization to complete or a time limit
  }
  Serial.println("Starting main program.");

  // Set up any pins
  pinMode(kLEDPin, OUTPUT);
  digitalWriteFast(kLEDPin, HIGH);  // Start with the LED on
  pinMode(kTXPin, OUTPUT);
  digitalWriteFast(kTXPin, LOW);

  // Initialize with the Null sketch
  currSketch = &nullSketch;
  currSketch->setup();

  Serial.println("Hello, DMX World!");
}

// Prints a sketch and prepends a star if it's the current active one.
// This indents by four spaces.
void printSketch(int sketchType, const char *s) {
  if (currSketch->type() == sketchType) {
    Serial.print("   *");
  } else {
    Serial.print("    ");
  }
  Serial.println(s);
}

// Main program loop.
void loop() {
  // Potentially choose a new sketch
  int avail = Serial.available();
  if (avail > 0) {
    int b = Serial.read();

    // First, transform aliases
    // and perform any commands
    switch (b) {
      case 'C':
        b = 'c';
        break;
      case 'F':
        b = 'f';
        break;
      case 'N':
        b = 'n';
        break;
      case '?':
        b = -1;
        Serial.println("Sketches:");
        printSketch('c', "(C)haser");
        printSketch('f', "(F)lasher");
        printSketch('n', "(N)ull");
        break;
    }

    if (b >= 0) {
      changeSketch(b);
    }
  }

  currSketch->loop();
}

// ---------------------------------------------------------------------------
//  Support functions
// ---------------------------------------------------------------------------

extern "C" void *_sbrk(int incr);

// Returns the amount of free RAM.
// See: https://forum.pjrc.com/threads/57269-I-can-t-seem-to-use-all-of-the-Teensy-4-0-s-memory
// See: https://github.com/arduino/ArduinoCore-API/issues/73
int freeRAM() {
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
  extern unsigned long _heap_end;
  intptr_t end = reinterpret_cast<intptr_t>(&_heap_end);
#else
  int v;
  intptr_t end = reinterpret_cast<intptr_t>(&v);
#endif  // __IMXRT1062__ || __IMXRT1052__
  return end - reinterpret_cast<intptr_t>(_sbrk(0));
}

void changeSketch(int sketchType) {
  if (sketchType == currSketch->type()) {
    return;
  }

  // Hold onto the old sketch so we can destroy it
  Sketch *oldSketch = currSketch;

  // Create a new sketch
  switch (sketchType) {
    case 'c':
      currSketch = &chaser;
      break;
    case 'f':
      currSketch = &flasher;
      break;
    case 'n':
      currSketch = &nullSketch;
      break;
    default:
      return;
  }

  Serial.printf("Changing sketch to: %s\n", currSketch->name().c_str());
  oldSketch->tearDown();
  currSketch->setup();
  Serial.printf("Free RAM: %d\n", freeRAM());
}

// ---------------------------------------------------------------------------
//  Sub-sketches
// ---------------------------------------------------------------------------

void NullSketch::setup() {
  digitalWriteFast(kLEDPin, HIGH);
}

Chaser::Chaser()
    : dmx_{uart},
      channel_(kChannel),
      lastChaseTimer_(0) {}

void Chaser::setup() {
  digitalWriteFast(kLEDPin, LOW);
  digitalWriteFast(kTXPin, HIGH);

  dmx_.set(1, 255);
  for (int i = 2; i < 513; i++) {
    dmx_.set(i, 0);
  }
  dmx_.setPacketSize(kPacketSize);
  dmx_.setRefreshRate(kRefreshRate);
  dmx_.begin();

  channel_ = 1;
  lastChaseTimer_ = 0;
}

void Chaser::tearDown() {
  digitalWriteFast(kTXPin, LOW);

  dmx_.end();
}

void Chaser::loop() {
  if (lastChaseTimer_ < kChaseInterval) {
    return;
  }
  lastChaseTimer_ = 0;

  // Set the current channel to zero, advance it, and then set the new channel
  // to 255
  dmx_.set(channel_, 0);
  channel_++;
  if (channel_ >= dmx_.packetSize()) {
    channel_ = 1;
  }
  dmx_.set(channel_, 255);

  // Blink the LED
  digitalWriteFast(kLEDPin, HIGH);
  delay(20);
  digitalWriteFast(kLEDPin, LOW);
}

Flasher::Flasher()
    : dmx_{uart},
      lastValue_(0),
      lastFrameTimer_(0),
      period_(kPeriodMax),
      phi_(0) {}

void Flasher::setup() {
  digitalWriteFast(kLEDPin, HIGH);  // Start with the LED on
  digitalWriteFast(kTXPin, LOW);

  dmx_.begin();
  lastFrameTimer_ = kTimeout;
}

void Flasher::tearDown() {
  digitalWriteFast(kLEDPin, LOW);

  dmx_.end();
}

void Flasher::loop() {
  uint8_t value;
  int read = dmx_.readPacket(&value, kChannel, 1);
  int64_t t = millis();
  if (read > 0) {
    // We've read everything we want to
    lastValue_ = value;
    lastFrameTimer_ = 0;

    // Use a wave equation to make the speed-ups and slow-downs smoother using
    // the offset, phi
    int32_t newPeriod = map(lastValue_, 0, 255, kPeriodMax, kPeriodMin);
    if (newPeriod != period_) {
      phi_ = (t*(period_ - newPeriod) + newPeriod*phi_)/period_;
      period_ = newPeriod;
    }
  }

  if (lastFrameTimer_ < kTimeout) {
    int32_t v = (t - phi_)%period_;
    if (v < period_/2) {
      digitalWriteFast(kLEDPin, HIGH);
    } else {
      digitalWriteFast(kLEDPin, LOW);
    }
  } else {
    digitalWriteFast(kLEDPin, LOW);
  }
}

#endif  // MAIN_TEST_PROGRAM
