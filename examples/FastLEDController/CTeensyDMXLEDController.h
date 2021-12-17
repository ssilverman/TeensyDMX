// FastLED controller for TeensyDMX.

/*
MIT License

Copyright (c) 2020-2021 Shawn Silverman

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// References:
// https://github.com/FastLED/FastLED/wiki/Multiple-Controller-Examples#managing-your-own-output
// https://forum.pjrc.com/threads/63231-FastLED-with-Teensy-4-1-fast-parallel-DMA-output-on-any-pin
// https://forum.pjrc.com/threads/65603-How-to-FastLED-with-TeensyDMX

// You could adapt this code to use a different DMX library, but it's
// suggested to change the name of the class (and file).

#ifndef CTEENSYDMXLEDCONTROLLER_H_
#define CTEENSYDMXLEDCONTROLLER_H_

#include <Arduino.h>
#include <FastLED.h>
#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// A FastLED LED controller implementation. It is assumed that all
// pixels are on consecutive channels, starting at the specified
// starting channel, and that each pixel is 3 bytes.
//
// For a 513-byte DMX packet, the maximum transmission rate is about
// 44Hz. A faster rate can be achieved by reducing the packet size to
// something smaller.
//
// The packet size template parameter includes the start code.
template <int START_CHANNEL = 1,
          EOrder RGB_ORDER = RGB,
          int PACKET_SIZE = 513>
class CTeensyDMXLEDController : public CPixelLEDController<RGB_ORDER> {
  static_assert(1 <= START_CHANNEL && START_CHANNEL <= 510,
                "Starting channel must be in the range [1, 510]");
  static_assert(START_CHANNEL < PACKET_SIZE && PACKET_SIZE <= 513,
                "Packet size must be in the range (starting channel, 513]");

 public:
  // Maximum number of pixels for this controller. This maximum exists
  // because there are only PACKET_SIZE-1 possible bytes in a DMX
  // packet, not including the start code.
  static constexpr int kMaxPixels = (PACKET_SIZE - START_CHANNEL)/3;

  // The last possible channel into which a pixel can go.
  // Recall that valid channels are 1-based.
  static constexpr int kMaxPixelChannel = START_CHANNEL + (kMaxPixels - 1)*3;

  // Creates a new controller and connects it to the specified
  // serial port.
  CTeensyDMXLEDController(HardwareSerial &uart)
      : CPixelLEDController<RGB_ORDER>(),
        dmxTx_{uart} {}

  ~CTeensyDMXLEDController() {
    dmxTx_.end();
  }

  // Initialize the controller by starting the transmitter.
  void init() override {
    dmxTx_.setPacketSize(PACKET_SIZE);
    dmxTx_.begin();
  }

  void clearLeds(int nLeds) override {
    dmxTx_.clear();
  }

 protected:
  // Send the pixels to the DMX transmitter.
  void showPixels(PixelController<RGB_ORDER> &pixels) override {
    int index = 0;
    while (pixels.has(1) && index < kMaxPixels*3) {
      // Set the pixel data
      packetBuf_[index++] = pixels.loadAndScale0();
      packetBuf_[index++] = pixels.loadAndScale1();
      packetBuf_[index++] = pixels.loadAndScale2();

      // Advance the pixels
      pixels.stepDithering();
      pixels.advanceData();
    }

    // This function sets all the values atomically
    dmxTx_.set(START_CHANNEL, packetBuf_, index);
  }

 private:
  teensydmx::Sender dmxTx_;  // DMX transmitter
  uint8_t packetBuf_[kMaxPixels * 3];
};

#endif  // CTEENSYDMXLEDCONTROLLER_H_
