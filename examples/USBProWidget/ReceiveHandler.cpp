// This file is part of the USBProWidget example in the TeensyDMX library.
// (c) 2019-2020 Shawn Silverman

#include "ReceiveHandler.h"

void sendDMXToHost(const uint8_t *buf, int len);

void ReceiveHandler::receivePacket(const uint8_t *buf, int len) {
  sendDMXToHost(buf, len);
}
