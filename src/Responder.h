// This file is part of the TeensyDMX library.
// (c) 2018-2022 Shawn Silverman

// Responder.h defines a generic interface for responding to special DMX
// packets in a Receiver.

#ifndef TEENSYDMX_RESPONDER_H_
#define TEENSYDMX_RESPONDER_H_

// C++ includes
#include <cstdint>

namespace qindesign {
namespace teensydmx {

// Responder defines an interface to a generic message responder that can
// respond to or process packets having a specific start code.
class Responder {
 public:
  // Gets the largest possible size that the output buffer can be when any
  // packet processing fills it in. If this is zero then the responder does not
  // respond with any data. This returns zero by default.
  virtual int outputBufferSize() const {
    return 0;
  }

  // Gets the BREAK time, in microseconds. This returns zero by default.
  virtual uint32_t breakTime() const {
    return 0;
  }

  // Gets the MARK after BREAK time, in microseconds. This returns zero
  // by default. The actual MAB time may be slightly longer.
  virtual uint32_t mabTime() const {
    return 0;
  }

  // Returns whether we should send a BREAK for the last valid response packet.
  // An output packet is considered valid if `processByte` returned a positive
  // value. The default implementation returns true.
  virtual bool isSendBreakForLastPacket() const {
    return true;
  }

  // Returns the delay, in microseconds, to wait before enabling the transmit
  // driver and sending a BREAK. A delay only happens if
  // `isSendBreakForLastPacket()` returns `true`. The default implementation
  // returns zero.
  virtual uint32_t preBreakDelay() const {
    return 0;
  }

  // Returns the delay, in microseconds, to wait before enabling the transmit
  // driver (and not sending a BREAK). This delay only happens if
  // `isSendBreakForLastPacket()` returns `false`. The default implementation
  // returns zero.
  virtual uint32_t preNoBreakDelay() const {
    return 0;
  }

  // Returns the delay, in microseconds, to wait before sending either a BREAK
  // or data. This occurs after either the `preBreakDelay()` or
  // `preNoBreakDelay()`. The default implementation returns zero.
  virtual uint32_t preDataDelay() const {
    return 0;
  }

  // Indicates whether this responder eats the packet and disallows
  // `Receiver::readPacket` from accessing the data, or does not eat it and
  // makes the data available to callers. The default is `true`.
  virtual bool eatPacket() const {
    return true;
  }

  // Processes single bytes as they are received. This returns a positive value
  // indicating the output buffer size if a response should be sent. It may be
  // assumed that `len` will start at zero and increment monotonically by one
  // for each call.
  //
  // A return value of -1 means that the packet is invalid, not understood, or
  // that a response is not possible or necessary. For example, in the RDM
  // protocol, a response is not possible if the device is muted or if we
  // received a packet sent to a broadcast address.
  //
  // In some cases, a BREAK should not be sent before a response packet. The
  // `isSendBreakForLastPacket()` function will indicate whether the last valid
  // output packet should have a BREAK sent before sending its data.
  //
  // Note that it can be assumed that `outBuf` has a size of at least the value
  // returned by `outputBufferSize()`.
  //
  // This may be called from inside an interrupt routine, so it's important to
  // execute as quickly as possible.
  //
  // @param buf a buffer containing the latest received byte
  // @param len the current accumulated length of the packet
  // @param outBuf buffer for output, at least `outputBufferSize()` bytes
  virtual int processByte(const uint8_t *buf, int len, uint8_t *outBuf) {
    return -1;
  }

  // Receives a packet. This doesn't return a response because the end of a
  // packet is determined heuristically and there's no way to guarantee that
  // response timing is correct.
  //
  // This is useful for processing packet data that doesn't require a response.
  //
  // The packet length will always be greater than zero because there will
  // always at least be a start code.
  //
  // This may be called from inside an interrupt routine, so it's important to
  // execute as quickly as possible.
  //
  // @param buf a buffer containing the packet
  // @param len the packet length, greater than zero, includes the start code
  virtual void receivePacket(const uint8_t *buf, int len) {}

 protected:
  Responder() = default;
  virtual ~Responder() = default;
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_RESPONDER_H_
