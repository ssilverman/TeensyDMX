// This file is part of the USBProWidget example in the TeensyDMX library.
// (c) 2019-2020 Shawn Silverman

// C++ includes
#include <cstdint>

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Handles received DMX packets.
class ReceiveHandler : public teensydmx::Responder {
 public:
  void receivePacket(const uint8_t *buf, int len) override;
};
