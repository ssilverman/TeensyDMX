// Responder.h defines a generic interface for responding to special DMX
// packets in a Receiver.

#ifndef QINDESIGN_RESPONDER_H_
#define QINDESIGN_RESPONDER_H_

// C++ includes
#include <cstddef>
#include <cstdint>

namespace qindesign {
namespace teensydmx {

// Responder defines an interface to a generic message responder that can
// respond to messages having a specific start code.
class Responder {
 public:
  Responder() = default;
  virtual ~Responder() = default;

  // Returns the start code used to detect whether we should use this
  // responder.
  virtual uint8_t getStartCode() const = 0;

  // Gets the largest possible size that the output buffer can be when
  // processPacket fills it in.
  virtual size_t getOutputBufferSize() const = 0;

  // Serial baud rates and formats
  virtual uint32_t getBreakBaud() const = 0;
  virtual uint32_t getBreakFormat() const = 0;
  virtual uint32_t getSlotsBaud() const = 0;
  virtual uint32_t getSlotsFormat() const = 0;

  // Processes single bytes as they are received. This returns a positive
  // value indicating the output buffer size if a response should be sent.
  // It may be assumed that index will start at zero and increment
  // monotonically by one for each call.
  //
  // A return value of -1 means that the packet is invalid, not understood,
  // or that a response is not possible or necessary. For example, in the RDM
  // protocol, a response is not possible if the device is muted or if we
  // received a packet sent to a broadcast address.
  //
  // In some rare cases, a BREAK should not be sent before a response packet.
  // The isSendBreakForLastPacket method will indicate whether the last
  // valid output packet should have a BREAK sent before sending its data.
  //
  // Note that it is expected that outBuf has a size of at least the value
  // returned by getOutputBufferSize().
  //
  // @param buf a buffer containing the latest received byte
  // @param the current accumulated length of the packet
  // @param outBuf buffer for output, at least getOutputBufferSize() bytes
  virtual int processByte(const uint8_t *buf, int len, uint8_t *outBuf) = 0;

  // Returns whether we should send a break for the last valid response
  // packet. An output packet is considered valid if processByte returned
  // a positive value. The default implementation returns true.
  virtual bool isSendBreakForLastPacket() const {
    return true;
  }

  // Returns the delay, in microseconds, to wait before enabling the
  // transmit driver and sending a break. A delay only happens if
  // isSendBreakForLastPacket returns true. The default implementation
  // returns zero.
  virtual uint32_t getPreBreakDelay() const {
    return 0;
  }

  // Returns the delay, in microseconds, to wait after (not sending a break
  // and) enabling the transmit driver. A delay only happens if
  // isSendBreakForLastPacket returns false. The default implementation
  // returns zero.
  virtual uint32_t getPreNoBreakDelay() const {
    return 0;
  }
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // QINDESIGN_RESPONDER_H_
