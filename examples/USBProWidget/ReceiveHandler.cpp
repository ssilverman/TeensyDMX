// This file is part of the USBProWidget example in the TeensyDMX library.
// (c) 2019-2020 Shawn Silverman

#include "ReceiveHandler.h"

// C++ includes
#include <cstring>

void sendDMX(const uint8_t *buf, int len);
void sendDMXChange(int channel, const uint8_t changeBits[5],
                   const uint8_t *data, int dataLen);

void ReceiveHandler::setSendOnChangeOnly(bool flag) {
  if (sendOnChangeOnly_ != flag) {
    if (flag) {
      lastLen_ = 0;
    }
    sendOnChangeOnly_ = flag;
  }
}

void ReceiveHandler::receivePacket(const uint8_t *buf, int len) {
  if (sendOnChangeOnly_ && len == lastLen_ && memcmp(buf, lastBuf_, len) == 0) {
    return;
  }
  lastLen_ = len;
  if (!sendOnChangeOnly_) {
    memcpy(lastBuf_, buf, len);
    sendDMX(buf, len);
    return;
  }

  int dataIndex;

  for (int i = 0; i < len; ) {
    if (buf[i] == lastBuf_[i]) {
      i++;
      continue;
    }

    // A mismatch, look at the next (up to) 40 bytes,
    // starting at the block-of-8 start
    dataIndex = 0;
    memset(changedBits_, 0, 5);
    int block = i / 8;
    i = block * 8;  // Reset 'i' to the start of this block
    for (int j = 0; j < 40 && i < len; j++) {
      if (buf[i] != lastBuf_[i]) {
        changedBits_[j / 8] |= uint8_t{1} << (j % 8);
        changedData_[dataIndex++] = buf[i];
      }
      i++;
    }
    sendDMXChange(block, changedBits_, changedData_, dataIndex);
  }
  memcpy(lastBuf_, buf, len);
}
