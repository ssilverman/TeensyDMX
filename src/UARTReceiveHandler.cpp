// This file is part of the TeensyDMX library.
// (c) 2019-2022 Shawn Silverman

#if defined(__MK20DX128__) || defined(__MK20DX256__) || \
    defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

#include "UARTReceiveHandler.h"

#include <core_pins.h>
#include <util/atomic.h>

namespace qindesign {
namespace teensydmx {

// RX control states
#define UART_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE | UART_C2_ILIE

extern const uint32_t kSlotsBaud;
extern const uint32_t kSlotsFormat;
extern const uint32_t kCharTime;  // In microseconds

void UARTReceiveHandler::start() {
  receiver_->uart_.begin(kSlotsBaud, kSlotsFormat);

#if defined(KINETISK)
  // Calculate the FIFO sizes now that the peripheral has been enabled and we can
  // access the registers
  if (!fifoSizesSet_) {
    // Calculate the FIFO sizes based on the RXFIFOSIZE and TXFIFOSIZE bits
    int bits = port_->PFIFO & 0x07;  // RXFIFOSIZE
    if (bits == 0 || bits == 7) {
      rxFIFOSize_ = 1;
    } else {
      rxFIFOSize_ = 1 << (bits + 1);
    }
    bits = (port_->PFIFO >> 4) & 0x07;  // TXFIFOSIZE
    if (bits == 0 || bits == 7) {
      txFIFOSize_ = 1;
    } else {
      txFIFOSize_ = 1 << (bits + 1);
    }

    fifoSizesSet_ = true;
  }
#endif  // KINETISK

  // Enable receive
  if (receiver_->txEnabled_) {
    port_->C2 = UART_C2_RX_ENABLE | UART_C2_TE;
  } else {
    port_->C2 = UART_C2_RX_ENABLE;
  }

  // Start counting IDLE after the start bit
  setILT(false);

  attachInterruptVector(irq_, irqHandler_);

  // Enable interrupt on frame error
  port_->C3 |= UART_C3_FEIE;
#if defined(KINETISK)
  attachInterruptVector(errorIRQ_, irqHandler_);
  // We fill bytes from the buffer in the framing error ISR, so we
  // can set to the same priority.
  NVIC_SET_PRIORITY(errorIRQ_, NVIC_GET_PRIORITY(irq_));
  NVIC_ENABLE_IRQ(errorIRQ_);
#endif  // KINETISK
}

#undef UART_C2_RX_ENABLE

void UARTReceiveHandler::end() const {
  receiver_->uart_.end();

#if defined(KINETISK)
  port_->C3 &= ~UART_C3_FEIE;
  NVIC_DISABLE_IRQ(errorIRQ_);
#endif  // KINETISK
}

void UARTReceiveHandler::setTXEnabled(bool flag) const {
  if (flag) {
    port_->C2 |= UART_C2_TE;
  } else {
    port_->C2 &= ~UART_C2_TE;
  }
}

void UARTReceiveHandler::setILT(bool flag) const {
  if (flag) {
    port_->C1 |= UART_C1_ILT;
  } else {
    port_->C1 &= ~UART_C1_ILT;
  }
}

void UARTReceiveHandler::setIRQState(bool flag) const {
#if defined(KINETISK)
  if (flag) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      NVIC_ENABLE_IRQ(irq_);
      NVIC_ENABLE_IRQ(errorIRQ_);
    }
  } else {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      NVIC_DISABLE_IRQ(irq_);
      NVIC_DISABLE_IRQ(errorIRQ_);
    }
  }
#else
  if (flag) {
    NVIC_ENABLE_IRQ(irq_);
  } else {
    NVIC_DISABLE_IRQ(irq_);
  }
#endif  // KINETISK
}

int UARTReceiveHandler::priority() const {
  return NVIC_GET_PRIORITY(irq_);
}

void UARTReceiveHandler::irqHandler() const {
  uint8_t status = port_->S1;

  uint32_t eventTime = micros();

  // A framing error likely indicates a BREAK, but it could also mean that there
  // were too few stop bits
  if ((status & UART_S1_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a BREAK.
    // A value of zero indicates a true BREAK and not some other
    // framing error.

#if defined(KINETISL)
    // Clear the flag
    if (serialIndex_ == 0) {
      port_->S1 |= UART_S1_FE;
    } else {
      // It's not necessary to read the data register here to clear the flag
      // because it's read next
      // port_->D;
    }
#endif  // KINETISL

#if defined(KINETISK)
    if (rxFIFOSize_ > 1) {
      // Flush anything in the buffer
      uint8_t avail = port_->RCFIFO;  // Receive Count
      if (avail > 1) {
        // Read everything but the last byte
        uint32_t timestamp = eventTime - kCharTime*avail;
        if (avail < port_->RWFIFO) {  // Receive Watermark
          timestamp -= kCharTime;
        }
        while (--avail > 0) {
          receiver_->receiveByte(port_->D, timestamp += kCharTime);
        }
      }
    }
#endif  // KINETISK

    if (port_->D == 0) {
      receiver_->receivePotentialBreak(eventTime);
    } else {
      receiver_->receiveBadBreak();
    }
    return;
  }

#if defined(KINETISK)
  if (rxFIFOSize_ > 1) {
    // If the receive buffer is full or there's an idle condition
    if ((status & (UART_S1_RDRF | UART_S1_IDLE)) != 0) {
      __disable_irq();
      uint8_t avail = port_->RCFIFO;  // Receive Count
      if (avail == 0) {
        // Read the register to clear the interrupt, but since it's empty,
        // this causes the FIFO to become misaligned, so send RXFLUSH to
        // reinitialize its pointers.
        // Do this inside no interrupts to avoid a potential race condition
        // between reading RCFIFO and flushing the FIFO.
        port_->D;
        port_->CFIFO = UART_CFIFO_RXFLUSH;
        __enable_irq();
        receiver_->receiveIdle(eventTime);
      } else {
        __enable_irq();
        bool idle = ((status & UART_S1_IDLE) != 0);
        uint32_t timestamp = eventTime - kCharTime*avail;
        if (avail < port_->RWFIFO) {  // Receive Watermark
          timestamp -= kCharTime;
        }
        // Read all but the last available, then read S1 and the final value
        // So says the chip docs,
        // Section 47.3.5 UART Status Register 1 (UART_S1)
        // In the NOTE part.
#if defined(__MK20DX128__) || defined(__MK20DX256__)
        bool errFlag = false;
#endif  // __MK20DX128__ || __MK20DX256__
        while (--avail > 0) {
#if defined(__MK20DX128__) || defined(__MK20DX256__)
          // Check that the 9th bit is high; used as the first stop bit
          if (!errFlag && (port_->C3 & UART_C3_R8) == 0) {
            errFlag = true;
            receiver_->receiveBadBreak();
          }
#endif  // __MK20DX128__ || __MK20DX256__
          receiver_->receiveByte(port_->D, timestamp += kCharTime);
        }
        port_->S1;
#if defined(__MK20DX128__) || defined(__MK20DX256__)
        if (!errFlag && (port_->C3 & UART_C3_R8) == 0) {
          receiver_->receiveBadBreak();
        }
#endif  // __MK20DX128__ || __MK20DX256__
        receiver_->receiveByte(port_->D, timestamp + kCharTime);
        if (idle) {  // Also capture any IDLE event
          receiver_->receiveIdle(eventTime);
          // The flag has been cleared by reading the data register
        }
      }
    }
  } else {  // No FIFO
    // If the receive buffer is full
    if ((status & UART_S1_RDRF) != 0) {
#if defined(__MK20DX128__) || defined(__MK20DX256__)
      // Check that the 9th bit is high; used as the first stop bit
      if ((port_->C3 & UART_C3_R8) == 0) {
        receiver_->receiveBadBreak();
      }
#endif  // __MK20DX128__ || __MK20DX256__
      receiver_->receiveByte(port_->D, eventTime);
    } else if ((status & UART_S1_IDLE) != 0) {
      receiver_->receiveIdle(eventTime);
      port_->D;  // Clear the flag
    }
  }
#else  // No FIFO
  // If the receive buffer is full
  if ((status & UART_S1_RDRF) != 0) {
    receiver_->receiveByte(port_->D, eventTime);
  } else if ((status & UART_S1_IDLE) != 0) {
    receiver_->receiveIdle(eventTime);

    // Clear the flag
    if (serialIndex_ == 0) {
      port_->S1 |= UART_S1_IDLE;
    } else {
      port_->D;
    }
  }
#endif  // KINETISK
}

void UARTReceiveHandler::txData(const uint8_t *b, int len) const {
  if (len <= 0) {
    return;
  }

  while (len > 0) {
    while ((port_->S1 & UART_S1_TDRE) == 0) {
      // Wait until we can transmit
    }
    port_->D = *(b++);
    len--;

#if defined(KINETISK)
    // Send the FIFO
    if (txFIFOSize_ > 1) {
      while (len > 0 && port_->TCFIFO < txFIFOSize_) {  // Transmit Count
        port_->S1;
        port_->D = *(b++);
        len--;
      }
    }
#endif // KINETISK
  }

  while ((port_->S1 & UART_S1_TC) == 0) {
    // Wait until transmission complete
  }
}

void UARTReceiveHandler::txBreak(uint32_t breakTime, uint32_t mabTime) const {
#if defined(KINETISK)
  if (txFIFOSize_ > 1) {
    while (port_->TCFIFO > 0) {  // Transmit Count
      // Wait for the FIFO to drain
    }
  }
#endif  // KINETISK

  while ((port_->S1 & UART_S1_TC) == 0) {
    // Wait until we can transmit
  }

  if (breakTime > 0) {
    port_->C3 |= UART_C3_TXINV;
    delayMicroseconds(breakTime);
    port_->C3 &= ~UART_C3_TXINV;
  }
  delayMicroseconds(mabTime);
}

}  // namespace teensydmx
}  // namespace qindesign

#endif  // __MK20DX128__ || __MK20DX256__ || __MKL26Z64__ || __MK64FX512__ ||
        // __MK66FX1M0__
