// RDM.cpp processes the RDM protocol.
// This is a simple test program.

#include "RDMResponder.h"

// C++ includes
#include <cstring>

namespace qindesign {
namespace teensydmx {

RDMResponder::RDMResponder(const uint8_t deviceUID[6])
    : packetValid_(false),
      totalPacketLen_(-1),
      sendBreakForLastPacket_(true) {}

int RDMResponder::processPacket(const uint8_t *buf, int bufLen, uint8_t *outBuf) {
  // Reset the "Send BREAK for last packet" state
  sendBreakForLastPacket_ = true;

  // Send a test response packet
  outBuf[0] = E120_SC_RDM;
  outBuf[1] = E120_SC_SUB_MESSAGE;
  outBuf[2] = 24;                  // Message length
  memcpy(&outBuf[3], &buf[9], 6);  // Destination UID
  memcpy(&outBuf[9], &buf[3], 6);  // Source UID
  outBuf[15] = buf[15];            // Transaction number
  outBuf[16] = E120_RESPONSE_TYPE_ACK;
  outBuf[17] = 0;                  // Message count
  outBuf[18] = 0;                  // Sub device (MSB)
  outBuf[19] = 0;                  // Sub device (LSB)
  outBuf[20] = E120_GET_COMMAND_RESPONSE;
  outBuf[21] = E120_SUPPORTED_PARAMETERS >> 8;
  outBuf[22] = E120_SUPPORTED_PARAMETERS;
  outBuf[23] = 0;                  // Parameter data length

  // Checksum
  uint16_t checksum = 0;
  for (int i = 0; i < 24; i++) {
    checksum += outBuf[i];
  }
  outBuf[24] = checksum >> 8;
  outBuf[25] = checksum;

  return 26;
}

int RDMResponder::processByte(const uint8_t *buf, int len, uint8_t *outBuf) {
  // Scan for this packet structure:
  // 0: E120_SC_RDM
  // 1: E120_SC_SUB_MESSAGE
  // 2: Message length (does not include 2-byte checksum)
  // 3-8: Destination UID
  // 9-14: Source UID
  // 15: Transaction number
  // 16: Port ID
  // 17: Message count (often zero)
  // 18-19: Sub-device
  // 20: Command class
  // 21-22: Parameter ID
  // 23: Parameter data length
  // 24-x: Parameter data
  // x+1-x+2: Checksum

  if (len == 1) {
    packetValid_ = true;
    totalPacketLen_ = -1;
  }
  if (!packetValid_) {
    return -1;
  }

  switch (len) {
    case 1:
      if (buf[0] != E120_SC_RDM) {
        packetValid_ = false;
        return -1;
      }
      break;
    case 2:
      if (buf[1] != E120_SC_SUB_MESSAGE) {
        packetValid_ = false;
        return -1;
      }
      break;
    case 3:
      totalPacketLen_ = buf[2] + 2;
      if (totalPacketLen_ < 5) {
        packetValid_ = false;
        return -1;
      }
      break;
    default:
      break;
  }

  if (totalPacketLen_ >= 0) {
    // The following check will also ignore packets that are larger
    // than expected
    if (len == totalPacketLen_) {
      digitalWriteFast(LED_BUILTIN, HIGH);
      int len = processPacket(buf, totalPacketLen_, outBuf);
      digitalWriteFast(LED_BUILTIN, LOW);
      return len;
    }
  }
  return -1;
}

}  // namespace teensydmx
}  // namespace qindesign
