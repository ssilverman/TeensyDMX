// This file is part of the SIPHandler example in the TeensyDMX library.
// (c) 2018-2020 Shawn Silverman

#ifndef SIPHANDLER_H_
#define SIPHANDLER_H_

// C++ includes
#include <cstdint>

// Other includes
#include <TeensyDMX.h>

namespace teensydmx = ::qindesign::teensydmx;

// Represents information in the latest SIP.
struct SIPData {
  uint8_t seq;          // Sequence number
  uint8_t universe;     // DMX universe
  uint8_t level;        // DMX Processing level
  uint8_t ver;          // Software version
  uint16_t packetLen;   // Standard packet length
  uint16_t numPackets;  // Packets sent since last SIP
  uint16_t mfrIDs[5];   // Manufacturing IDs

  bool checksumValid = false;
  bool hasLastPacket = false;  // Whether there was data to check
};

// SIPHandler processes System Information Packets (SIP) together with
// other packets. Retrieve packet information here instead of from the
// Receiver instance.
//
// If requested, regular (NULL start code) packets are held until the
// next SIP is received and the checksum validated.
class SIPHandler final : public teensydmx::Responder {
 public:
  static constexpr uint8_t startCode() { return 0xcf; }

  // Initialize the object.
  SIPHandler()
      : state_{States::kImmediate},
        packet_{0},
        packetSize_(0),
        held_(false),
        sipData_{0},
        sipDataValid_(false) {}

  // Gets packet data. This will return -1 if no packets or invalid
  // packets are being received, otherwise this will return the number
  // of bytes read. Once a packet is read, this will return -1 until
  // a new packet is received.
  //
  // Held packets are not released until the next SIP comes in with a
  // valid checksum. If a SIP checksum does not match, then the packet
  // is discarded.
  int readPacket(uint8_t *buf, int startChannel, int len);

  // Gets the latest SIP data. This returns false if there is no SIP
  // data or if the parameter 'sipData' is nullptr. Otherwise, this
  // returns true.
  bool getSIPData(SIPData *sipData) {
    if (!sipDataValid_ || sipData == nullptr) {
      return false;
    }

    // Ideally, we should define an `operator=` that takes a volatile
    // parameter instead of using `const_cast`
    *sipData = *const_cast<SIPData *>(&sipData_);
    return true;
  }

  // A packet was just received by the receiver. This is implemented
  // similarly to Receiver::receivePacket.
  void receivePacket(const uint8_t *buf, int len) override;

 private:
  // Parsing states.
  enum class States {
    kImmediate,  // Immediate use
    kHoldNext,   // Hold the next regular packet
    kSIPCheck,   // Expect a SIP and not a regular packet next
  };

  States state_;

  // Packet data
  volatile uint8_t packet_[teensydmx::kMaxDMXPacketSize];
  volatile int packetSize_;
  volatile bool held_;

  // SIP data
  volatile SIPData sipData_;
  volatile bool sipDataValid_;  // Indicates that at least one SIP
                                // was received
};

#endif  // SIPHANDLER_H_
