// This file is part of the TeensyDMX library.
// (c) 2019-2023 Shawn Silverman

#if defined(__IMXRT1062__) || defined(__IMXRT1052__) || defined(__MK66FX1M0__)

#include "LPUARTReceiveHandler.h"

#include <core_pins.h>

namespace qindesign {
namespace teensydmx {

// RX control states
#define LPUART_CTRL_RX_ENABLE \
  LPUART_CTRL_RE | LPUART_CTRL_RIE | LPUART_CTRL_ILIE

extern const uint32_t kSlotsBaud;
extern const uint32_t kSlotsFormat;
extern const uint32_t kCharTime;  // In microseconds

void LPUARTReceiveHandler::start() {
  receiver_->uart_.begin(kSlotsBaud, kSlotsFormat);

#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
  // Calculate the FIFO size now that the peripheral has been enabled and we can
  // access the registers
  if (!txFIFOSizeSet_) {
    // Calculate the TX FIFO size based on the TXFIFOSIZE bits
    int bits = (port_->FIFO >> 4) & 0x07;  // TXFIFOSIZE
    if (bits == 0) {
      txFIFOSize_ = 1;
    } else {
      txFIFOSize_ = 1 << (bits + 1);
    }

    txFIFOSizeSet_ = true;
  }
#endif  // __IMXRT1062__ || __IMXRT1052__

  // Enable receive and interrupt on frame error
  if (receiver_->txEnabled_) {
    port_->CTRL = LPUART_CTRL_RX_ENABLE | LPUART_CTRL_FEIE | LPUART_CTRL_TE;
  } else {
    port_->CTRL = LPUART_CTRL_RX_ENABLE | LPUART_CTRL_FEIE;
  }

  // Start counting IDLE after the start bit
  setILT(false);

  attachInterruptVector(irq_, irqHandler_);
}

#undef LPUART_CTRL_RX_ENABLE

void LPUARTReceiveHandler::end() const {
  receiver_->uart_.end();
}

void LPUARTReceiveHandler::setTXEnabled(bool flag) const {
  if (flag) {
    port_->CTRL |= LPUART_CTRL_TE;
  } else {
    port_->CTRL &= ~LPUART_CTRL_TE;
  }
}

void LPUARTReceiveHandler::setILT(bool flag) const {
  if (flag) {
    port_->CTRL |= LPUART_CTRL_ILT;
  } else {
    port_->CTRL &= ~LPUART_CTRL_ILT;
  }
}

void LPUARTReceiveHandler::setIRQState(bool flag) const {
  if (flag) {
    NVIC_ENABLE_IRQ(irq_);
  } else {
    NVIC_DISABLE_IRQ(irq_);
  }
}

int LPUARTReceiveHandler::priority() const {
  return NVIC_GET_PRIORITY(irq_);
}

void LPUARTReceiveHandler::irqHandler() const {
  uint32_t status = port_->STAT;

  uint32_t eventTime = micros();

  // A framing error likely indicates a BREAK, but it could also mean that there
  // were too few stop bits
  if ((status & LPUART_STAT_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a BREAK.
    // A value of zero indicates a true BREAK and not some other
    // framing error.

    // Clear interrupt flags
    port_->STAT |= (LPUART_STAT_FE | LPUART_STAT_IDLE);

#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
    // Flush anything in the buffer
    uint8_t avail = (port_->WATER >> 24) & 0x07;  // RXCOUNT
    if (avail > 1) {
      // Read everything but the last byte
      uint32_t timestamp = eventTime - kCharTime*avail;
      while (--avail > 0) {
        receiver_->receiveByte(port_->DATA, timestamp += kCharTime);
      }
    }
#endif  // __IMXRT1062__ || __IMXRT1052__

    // 32-bit data, so only look at the bottom 8 bits
    if ((port_->DATA & 0xff) == 0) {
      receiver_->receivePotentialBreak(eventTime);
    } else {
      receiver_->receiveBadBreak();
    }
    return;
  }

#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
  // If the receive buffer is full or there's an idle condition
  if ((status & (LPUART_STAT_RDRF | LPUART_STAT_IDLE)) != 0) {
    uint8_t avail = (port_->WATER >> 24) & 0x07;  // RXCOUNT
    if (avail == 0) {
      receiver_->receiveIdle(eventTime);
      if ((status & LPUART_STAT_IDLE) != 0) {
        port_->STAT |= LPUART_STAT_IDLE;  // Clear the flag
      }
    } else {
      bool idle = ((status & LPUART_STAT_IDLE) != 0);
      uint32_t timestamp = eventTime - kCharTime*avail;
      if (avail < ((port_->WATER >> 16) & 0x03)) {  // RXWATER
        timestamp -= kCharTime;
      }
      while (avail-- > 0) {
        receiver_->receiveByte(port_->DATA, timestamp += kCharTime);
      }
      if (idle) {  // Also capture any IDLE event
        receiver_->receiveIdle(eventTime);
        port_->STAT |= LPUART_STAT_IDLE;  // Clear the flag
      }
    }
  }
#else  // No FIFO
  // If the receive buffer is full
  if ((status & LPUART_STAT_RDRF) != 0) {
    receiver_->receiveByte(port_->DATA, eventTime);
  } else if ((status & LPUART_STAT_IDLE) != 0) {
    receiver_->receiveIdle(eventTime);
    port_->STAT |= LPUART_STAT_IDLE;  // Clear the flag
  }
#endif  // __IMXRT1062__ || __IMXRT1052__
}

void LPUARTReceiveHandler::txData(const uint8_t *b, int len) const {
  if (len <= 0) {
    return;
  }

  while (len > 0) {
    while ((port_->STAT & LPUART_STAT_TDRE) == 0) {
      // Wait until we can transmit
    }
    port_->DATA = *(b++);
    len--;

#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
    // Send the FIFO
    while (len > 0 && ((port_->WATER >> 8) & 0x07) < txFIFOSize_) {  // TXCOUNT
      port_->DATA = *(b++);
      len--;
    }
#endif  // __IMXRT1062__ || __IMXRT1052__
  }

  while ((port_->STAT & LPUART_STAT_TC) == 0) {
    // Wait until transmission complete
  }
}

void LPUARTReceiveHandler::txBreak(uint32_t breakTime, uint32_t mabTime) const {
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
  while (((port_->WATER >> 8) & 0x07) > 0) {  // TXCOUNT
    // Wait for the FIFO to drain
  }
#endif  // __IMXRT1062__ || __IMXRT1052__

  while ((port_->STAT & LPUART_STAT_TC) == 0) {
    // Wait until we can transmit
  }

  if (breakTime > 0) {
    port_->CTRL |= LPUART_CTRL_TXINV;
    delayMicroseconds(breakTime);
    port_->CTRL &= ~LPUART_CTRL_TXINV;
  }
  delayMicroseconds(mabTime);
}

}  // namespace teensydmx
}  // namespace qindesign

#endif  // __IMXRT1062__ || __IMXRT1052__ || __MK66FX1M0__
