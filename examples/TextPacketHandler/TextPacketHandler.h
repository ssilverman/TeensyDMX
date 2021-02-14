// This file is part of the TextPacketHandler example in the TeensyDMX library.
// (c) 2018-2020 Shawn Silverman

#ifndef TEXTPACKETHANDLER_H_
#define TEXTPACKETHANDLER_H_

// C++ includes
#include <cstdint>

// Other includes
#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// These functions are defined elsewhere. Note that they
// could be defined by passing function pointers to the
// TextPacketHandler object.
//
// page: the page number
// charsPerLine: number of characters per line, for formatting,
//               zero to ignore
// text: the NUL-terminated string
// len: the string length, not including the terminating NUL
void setText(uint8_t page, uint8_t charsPerLine, const char *text, int len);
void setUTF8Text(uint8_t page, uint8_t charsPerLine, const char *text, int len);

// TextPacketHandler processes ASCII and UTF-8 Text Packets.
class TextPacketHandler final : public teensydmx::Responder {
 public:
  // Supported start codes
  // For fun, there's more than one
  static constexpr uint8_t kASCII = 0x17;
  static constexpr uint8_t kUTF8 = 0x90;
  static constexpr uint8_t kStartCodes[]{kASCII, kUTF8};

  // A packet was just received by the receiver.
  void receivePacket(const uint8_t *buf, int len) override;
};

#endif  // TEXTPACKETHANDLER_H_
