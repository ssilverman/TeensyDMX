// This file is part of the TextPacketHandler example in the TeensyDMX library.
// (c) 2018-2019 Shawn Silverman

#include "TextPacketHandler.h"

// Define this here (C++17 doesn't need this but earlier versions do).
constexpr uint8_t TextPacketHandler::kStartCodes[];

void TextPacketHandler::receivePacket(const uint8_t *buf, int len) {
    // The packet must contain at least 3 bytes (plus the start code)
    if (len < 4) {
      return;
    }
    uint8_t page = buf[1];
    uint8_t charsPerLine = buf[2];

    // Look for the NUL character
    int nulIndex = -1;
    for (int i = 3; i < len; i++) {
      if (buf[i] == 0) {
        nulIndex = i;
        break;
      }
    }
    if (nulIndex < 0) {
      // Ignore packets missing a NUL character
      return;
    }

    switch (buf[0]) {
      case kASCII:
        setText(page, charsPerLine,
                reinterpret_cast<const char *>(&buf[3]), nulIndex - 3);
        break;
      case kUTF8:
        // TODO: Validate the UTF-8 text
        setUTF8Text(page, charsPerLine,
                    reinterpret_cast<const char *>(&buf[3]), nulIndex - 3);
        break;
    }
}
