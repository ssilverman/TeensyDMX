// Basic main program, so it's easy to compile the project.
//
// This uses the concept of sub-sketches, where serial input selects which
// sub-sketch to run. The initial running sketch is the Null sketch; it does
// nothing. The other sketches are:
// 1. Chaser: This cycles an output of 255 on each channel.
// 2. Flasher: Receives input and flashes an LED with a period proportional to
//    the input.
//
// (c) 2018-2019 Shawn Silverman

// Define MAIN_TEST_PROGRAM to use this test program.
#ifdef MAIN_TEST_PROGRAM

// C++ includes
#include <cstddef>
#include <cstdint>
#include <memory>

// Other includes
#include <Arduino.h>

// Project includes
#include "TeensyDMX.h"

namespace teensydmx = ::qindesign::teensydmx;

// Basic sub-sketch.
class Sketch {
 public:
  Sketch() = default;
  virtual ~Sketch() = default;

  virtual void setup() {}
  virtual void tearDown() {}
  virtual void loop() {}
  virtual String name() = 0;
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

 private:
  static constexpr int kChannel = 1;
  static constexpr unsigned long kChaseInterval = 1000;  // 1s

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

 private:
  static constexpr int kChannel = 1;
  static constexpr unsigned long kTimeout = 1000;

  // Minimum and maximum frequencies, expressed as a period in milliseconds
  static constexpr int32_t kPeriodMax = 1000;
  static constexpr int32_t kPeriodMin = 30;

  teensydmx::Receiver dmx_;
  uint8_t buf_[1];
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
std::unique_ptr<Sketch> currSketch{std::make_unique<NullSketch>()};
int currSketchType = -1;

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
  changeSketch('n');

  Serial.println("Hello, DMX World!");
}

// Main program loop.
void loop() {
  // Potentially choose a new sketch
  int avail = Serial.available();
  if (avail > 0) {
    changeSketch(Serial.read());
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
#endif
  return end - reinterpret_cast<intptr_t>(_sbrk(0));
}

void changeSketch(int sketchType) {
  if (sketchType == currSketchType) {
    return;
  }

  // Check for a valid sketch type
  switch (sketchType) {
    case 'c':
    case 'f':
    case 'n':
      break;
    default:
      return;
  }

  // Destroy any current sketch
  currSketch->tearDown();

  // Create a new sketch
  switch (sketchType) {
    case 'c':
      currSketch.reset(new Chaser{});
      break;
    case 'f':
      currSketch.reset(new Flasher{});
      break;
    case 'n':
      currSketch.reset(new NullSketch{});
      break;
  }
  currSketchType = sketchType;

  Serial.printf("Changing sketch to: %s\n", currSketch->name().c_str());
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
  if (channel_ > 512) {
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
  int read = dmx_.readPacket(buf_, kChannel, 1);
  int64_t t = millis();
  if (read > 0) {
    // We've read everything we want to
    lastValue_ = buf_[0];
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
