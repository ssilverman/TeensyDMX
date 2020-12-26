// This file is part of the SIPHandler example in the TeensyDMX library.
// (c) 2018-2020 Shawn Silverman

#include "SIPHandler.h"

// C++ includes
#include <algorithm>
#include <numeric>

// Gets a uint16_t value from the given array.
uint16_t getUint16(const uint8_t *b) {
  return (uint16_t{b[0]} << 8) | uint16_t{b[1]};
}

// Checks the SIP data for basic correctness.
bool checkSIP(const uint8_t *buf, int len) {
  if (buf[0] != SIPHandler::startCode() || len != 25) {
    return false;
  }
  uint8_t checksum = 0;
  for (int i = 0; i < 25; i++) {
    checksum += buf[i];
  }
  checksum = ~checksum;
  if (checksum != buf[24]) {
    return false;
  }

  return true;
}

void SIPHandler::receivePacket(const uint8_t *buf, int len) {
  bool copyData = true;

  if (buf[0] == startCode()) {  // Process a SIP
    if (!checkSIP(buf, len)) {
      // Discard this packet and any held packet
      // TODO: Is it the right choice, to discard the held packet?
      state_ = States::kImmediate;
      held_ = false;

      // Copy the packet data, maybe the next SIP will use this?
      std::copy_n(&buf[0], len, &packet_[0]);
      packetSize_ = len;
      return;
    }

    // Store the SIP data
    sipData_.seq        = buf[5];
    sipData_.universe   = buf[6];
    sipData_.level      = buf[7];
    sipData_.ver        = buf[8];
    sipData_.packetLen  = getUint16(&buf[9]);
    sipData_.numPackets = getUint16(&buf[11]);
    for (int i = 0; i < 5; i++) {
      sipData_.mfrIDs[i] = getUint16(&buf[13 + i*2]);
    }
    sipData_.checksumValid = false;

    // Check the checksum of the last packet
    uint16_t sipCheck = getUint16(&buf[3]);
    if (packetSize_ > 0) {
      uint16_t check =
          ~std::accumulate(&packet_[0], &packet_[packetSize_], uint16_t{0});
      sipData_.checksumValid = (sipCheck == check);
      sipData_.hasLastPacket = true;
    }

    sipDataValid_ = true;

    switch (state_) {
      case States::kImmediate:
        if ((buf[2] & 0x01) != 0) {  // If hold-next
          state_ = States::kHoldNext;
        }
        break;
      case States::kHoldNext:
        if ((buf[2] & 0x01) == 0) {  // If not hold-next
          state_ = States::kImmediate;
        }
        break;
      case States::kSIPCheck: {  // Check the held regular packet
        // Only copy the held packet if the checksum matches
        if (!sipData_.checksumValid) {
          packetSize_ = 0;
        }
        held_ = false;

        // Don't copy the received data because we don't want to
        // overwrite any held data, including no data
        copyData = false;

        // Manage the hold state
        if ((buf[2] & 0x01) != 0) {
          state_ = States::kHoldNext;
        } else {
          state_ = States::kImmediate;
        }
        break;
      }
    }

    // Instead of returning, store the packet data so that
    // its checksum can also be checked
  } else if (buf[0] == 0) {  // Process a regular packet
    switch(state_) {
      case States::kImmediate:
        break;
      case States::kHoldNext:  // Hold this regular packet until the
                               // next SIP
        held_ = true;
        state_ = States::kSIPCheck;
        return;
      case States::kSIPCheck:
        held_ = false;
        state_ = States::kImmediate;
        break;
    }
  } else {  // Another alternate start code
    // Go back to the immediate state if another packet has come
    // between a held regular packet and the next SIP.
    // Note that this logic will never get called if this responder
    // is not registered for other start codes.
    switch (state_) {
      case States::kImmediate:
        break;
      case States::kHoldNext:
        // We only need to hold regular packets
        // Keeping the state intact here means that the next regular
        // packet will be held, even though it doesn't come
        // immediately after the SIP that requested the packet to
        // be held
        break;
      case States::kSIPCheck:
        // We've held onto the previous packet, so the next packet
        // needs to be a SIP or regular packet
        held_ = false;
        state_ = States::kImmediate;
        break;
    }
  }

  if (copyData) {
    std::copy_n(&buf[0], len, &packet_[0]);
    packetSize_ = len;
  }
}

int SIPHandler::readPacket(uint8_t *buf, int startChannel, int len) {
  // Also don't read packets if the stored packet is a SIP
  if (held_ || packetSize_ <= 0 || packet_[0] == startCode()) {
    return -1;
  }
  if (len <= 0 || startChannel < 0 || packetSize_ <= startChannel) {
    return 0;
  }

  int retval = -1;
  noInterrupts();
  if (!held_ && packetSize_ > 0 && packet_[0] != startCode()) {
    if (startChannel >= packetSize_) {
      retval = 0;
    } else {
      if (startChannel + len > packetSize_) {
        len = packetSize_ - startChannel;
      }
      // NOTE: std::copy_n can take a volatile source
      std::copy_n(&packet_[startChannel], len, &buf[0]);
      retval = len;
    }
    packetSize_ = 0;
  }
  interrupts();
  return retval;
}
