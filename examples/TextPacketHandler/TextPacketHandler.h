#ifndef TEXTPACKETHANDLER_H_
#define TEXTPACKETHANDLER_H_

// C++ includes
#include <cstdint>

// Other includes
#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// TextPacketHandler processes ASCII and UTF-8 Text Packets.
class TextPacketHandler final : public teensydmx::Responder {
 public:
  // Supported start codes
  // For fun, there's more than one
  static constexpr uint8_t kASCII = 0x17;
  static constexpr uint8_t kUTF8 = 0x90;
  static constexpr uint8_t kStartCodes[]{kASCII, kUTF8};

  // Constructor, initialize the object.
  TextPacketHandler() : teensydmx::Responder() {}

 protected:
  // A packet was just received by the receiver.
  void receivePacket(const uint8_t *buf, int len) override;
};

#endif  // TEXTPACKETHANDLER_H_
