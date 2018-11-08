// RDM.h contains definitions for the RDM implementation.

#ifndef QINDESIGN_RDM_H_
#define QINDESIGN_RDM_H_

// C++ includes
#include <cstdint>

// Other includes
#include <Arduino.h>

// Project includes
#include "Responder.h"
#include "rdmconst.h"

namespace qindesign {
namespace teensydmx {

// Implements RDM handling.
class RDMResponder final : public Responder {
 public:
  // Creates a new RDM context with the given device UID. The top two bytes
  // are the manufacturer ID.
  RDMResponder(const uint8_t deviceUID[6]);

  ~RDMResponder() override = default;

  uint8_t getStartCode() const override {
    return E120_SC_RDM;
  }

  size_t getOutputBufferSize() const override {
    return kOutputBufSize;
  }

  uint32_t getBreakBaud() const override {
    return kBreakBaud;
  }

  uint32_t getBreakFormat() const override {
    return kBreakFormat;
  }

  uint32_t getSlotsBaud() const override {
    return kSlotsBaud;
  }

  uint32_t getSlotsFormat() const override {
    return kSlotsFormat;
  }

  // Processes single bytes as they are received. When a complete RDM packet
  // is received, this passes the packet to processPacket.
  int processByte(const uint8_t *buf, int len, uint8_t *outBuf) override;

  bool isSendBreakForLastPacket() const override {
    return sendBreakForLastPacket_;
  }

  uint32_t getPreBreakDelay() const override {
    return kPreBreakDelay;
  }

  uint32_t getPreNoBreakDelay() const override {
    return kPreNoBreakDelay;
  }

 private:
  static constexpr size_t kOutputBufSize = 257;
  static constexpr uint32_t kPreBreakDelay = 176;  // In microseconds
  static constexpr uint32_t kPreNoBreakDelay = 4;  // In microseconds

  // RDM timing:
  // 50000 baud, 8N1: 180us break, 20us MAB
  // 45500 baud, 8E1: 220us break, 22us MAB
  static constexpr uint32_t kBreakBaud   = 50000;
  static constexpr uint32_t kBreakFormat = SERIAL_8N1;
  static constexpr uint32_t kSlotsBaud   = 250000;
  static constexpr uint32_t kSlotsFormat = SERIAL_8N2;

  static constexpr uint16_t kRDMProtocolVersion = 0x0100;

  // Processes a single RDM packet. This returns a positive value if data
  // was put into outBuf and it's ready for transmission. This returns -1
  // if no response is to be sent.
  int processPacket(const uint8_t *buf, int len, uint8_t *outBuf);

  // Packet accumulation state
  bool packetValid_;  // Indicates whether the current packet data is valid
  int totalPacketLen_;  // The total expected packet size

  // Other state
  bool sendBreakForLastPacket_;
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // QINDESIGN_RDM_H_
