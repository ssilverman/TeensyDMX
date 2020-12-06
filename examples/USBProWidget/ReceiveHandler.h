// This file is part of the USBProWidget example in the TeensyDMX library.
// (c) 2019-2020 Shawn Silverman

// C++ includes
#include <cstdint>

#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Handles received DMX packets.
class ReceiveHandler : public teensydmx::Responder {
 public:
  explicit ReceiveHandler(Stream &stream)
      : stream_(stream_),
        sendOnChangeOnly_(false),
        lastBuf_{0},
        lastLen_(-1) {}

  // Sets this mode. If the mode is changed to true then the initial data is
  // reset to zero length.
  void setSendOnChangeOnly(bool flag);

 protected:
  void receivePacket(const uint8_t *buf, int len) override;

  Stream &stream_;

  volatile bool sendOnChangeOnly_;
  uint8_t lastBuf_[513];
  volatile int lastLen_;

  // For sending DMX Change messages
  uint8_t changedBits_[5];
  uint8_t changedData_[40];
};
