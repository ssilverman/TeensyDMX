#include "TeensyDMX.h"

// C++ includes
#include <cstring>

namespace qindesign {
namespace teensydmx {

constexpr uint32_t kSlotsBaud = 250000;
constexpr uint32_t kSlotsFormat = SERIAL_8N2;

// RX control states
#ifdef HAS_KINETISK_UART0_FIFO
#define UART0_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE | UART_C2_ILIE
#else
#define UART0_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE
#endif  // HAS_KINETISK_UART0_FIFO
#ifdef HAS_KINETISK_UART1_FIFO
#define UART1_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE | UART_C2_ILIE
#else
#define UART1_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE
#endif  // HAS_KINETISK_UART1_FIFO
#ifdef HAS_KINETISK_UART2_FIFO
#define UART2_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE | UART_C2_ILIE
#else
#define UART2_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE
#endif  // HAS_KINETISK_UART2_FIFO

#ifdef HAS_KINETISK_UART3
#define UART3_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
#define UART4_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE
#endif  // HAS_KINETISK_UART4
#ifdef HAS_KINETISK_UART5
#define UART5_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE
#endif  // HAS_KINETISK_UART5
#ifdef HAS_KINETISK_LPUART0
#define LPUART0_CTRL_RX_ENABLE LPUART_CTRL_RE | LPUART_CTRL_RIE
#endif  // HAS_KINETISK_LPUART0

// Used by the RX ISR's.
Receiver *rxInstances[6]{nullptr};

void Receiver::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  if (serialIndex_ < 0) {
    return;
  }

  // Set up the instance for the ISR's
  if (rxInstances[serialIndex_] != nullptr) {
    rxInstances[serialIndex_]->end();
  }
  rxInstances[serialIndex_] = this;

  uart_.begin(kSlotsBaud, kSlotsFormat);

  switch (serialIndex_) {
    case 0:
      // Enable receive-only
      UART0_C2 = UART0_C2_RX_ENABLE;

      attachInterruptVector(IRQ_UART0_STATUS, uart0_rx_status_isr);
      attachInterruptVector(IRQ_UART0_ERROR, uart0_rx_error_isr);

      // We fill bytes from the buffer in the framing error ISR, so we
      // can set to the same priority.
      NVIC_SET_PRIORITY(IRQ_UART0_ERROR, NVIC_GET_PRIORITY(IRQ_UART0_STATUS));

      // Enable UART0 interrupt on frame error
      UART0_C3 |= UART_C3_FEIE;
      NVIC_ENABLE_IRQ(IRQ_UART0_ERROR);
      break;

    case 1:
      UART1_C2 = UART1_C2_RX_ENABLE;
      attachInterruptVector(IRQ_UART1_STATUS, uart1_rx_status_isr);
      attachInterruptVector(IRQ_UART1_ERROR, uart1_rx_error_isr);
      NVIC_SET_PRIORITY(IRQ_UART1_ERROR, NVIC_GET_PRIORITY(IRQ_UART1_STATUS));
      UART1_C3 |= UART_C3_FEIE;
      NVIC_ENABLE_IRQ(IRQ_UART1_ERROR);
      break;

    case 2:
      UART2_C2 = UART2_C2_RX_ENABLE;
      attachInterruptVector(IRQ_UART2_STATUS, uart2_rx_status_isr);
      attachInterruptVector(IRQ_UART2_ERROR, uart2_rx_error_isr);
      NVIC_SET_PRIORITY(IRQ_UART2_ERROR, NVIC_GET_PRIORITY(IRQ_UART2_STATUS));
      UART2_C3 |= UART_C3_FEIE;
      NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);
      break;

#ifdef HAS_KINETISK_UART3
    case 3:
      UART3_C2 = UART3_C2_RX_ENABLE;
      attachInterruptVector(IRQ_UART3_STATUS, uart3_rx_status_isr);
      attachInterruptVector(IRQ_UART3_ERROR, uart3_rx_error_isr);
      NVIC_SET_PRIORITY(IRQ_UART3_ERROR, NVIC_GET_PRIORITY(IRQ_UART3_STATUS));
      UART3_C3 |= UART_C3_FEIE;
      NVIC_ENABLE_IRQ(IRQ_UART3_ERROR);
      break;
#endif  // HAS_KINETISK_UART3

#ifdef HAS_KINETISK_UART4
    case 4:
      UART4_C2 = UART4_C2_RX_ENABLE;
      attachInterruptVector(IRQ_UART4_STATUS, uart4_rx_status_isr);
      attachInterruptVector(IRQ_UART4_ERROR, uart4_rx_error_isr);
      NVIC_SET_PRIORITY(IRQ_UART4_ERROR, NVIC_GET_PRIORITY(IRQ_UART4_STATUS));
      UART4_C3 |= UART_C3_FEIE;
      NVIC_ENABLE_IRQ(IRQ_UART4_ERROR);
      break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
    case 5:
      UART5_C2 = UART5_C2_RX_ENABLE;
      attachInterruptVector(IRQ_UART5_STATUS, uart5_rx_status_isr);
      attachInterruptVector(IRQ_UART5_ERROR, uart5_rx_error_isr);
      NVIC_SET_PRIORITY(IRQ_UART5_ERROR, NVIC_GET_PRIORITY(IRQ_UART5_STATUS));
      UART5_C3 |= UART_C3_FEIE;
      NVIC_ENABLE_IRQ(IRQ_UART5_ERROR);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      LPUART0_CTRL = LPUART0_CTRL_RX_ENABLE | LPUART_CTRL_FEIE;
      attachInterruptVector(IRQ_LPUART0, lpuart0_rx_isr);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }

  activeBuf_ = buf1_;
  inactiveBuf_ = buf2_;
}

void Receiver::end() {
  if (!began_) {
    return;
  }
  began_ = false;

  if (serialIndex_ < 0) {
    return;
  }

  // Remove the reference from the instances
  rxInstances[serialIndex_] = nullptr;

  uart_.end();

  switch (serialIndex_) {
    case 0:
      // Disable UART0 interrupt on frame error
      UART0_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
      break;
    case 1:
      UART1_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
      break;
    case 2:
      UART2_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
      break;
#ifdef HAS_KINETISK_UART3
    case 3:
      UART3_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART3_ERROR);
      break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
    case 4:
      UART4_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART4_ERROR);
      break;
#endif  // HAS_KINETISK_UART4
#ifdef HAS_KINETISK_UART5
    case 5:
      UART5_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART5_ERROR);
      break;
#endif  // HAS_KINETISK_UART5
// NOTE: Nothing needed for LPUART0
  }
}

// memcpy implementation that accepts a const volatile source.
// Derived from:
// https://github.com/ARM-software/arm-trusted-firmware/blob/master/lib/stdlib/mem.c
static void *memcpy(void *dst, const volatile void *src, size_t len) {
  const volatile uint8_t *s = reinterpret_cast<const volatile uint8_t *>(src);
  uint8_t *d = reinterpret_cast<uint8_t *>(dst);

  while (len-- != 0) {
    *(d++) = *(s++);
  }

  return dst;
}

int Receiver::readPacket(uint8_t *buf, int startChannel, int len) {
  if (packetSize_ <= 0) {
    return -1;
  }
  if (len <= 0 || startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return 0;
  }

  int retval = -1;
  disableIRQs();
  //{
    // Instead of using a timer, we can use this function to poll timeouts
    if (inPacket_) {
      if ((millis() - lastBreakTime_) > kMaxDMXPacketTime) {
        packetTimeoutCount_++;
        completePacket();
      }
    }

    if (packetSize_ > 0) {
      if (startChannel >= packetSize_) {
        retval = 0;
      } else {
        if (startChannel + len > packetSize_) {
          len = packetSize_ - startChannel;
        }
        memcpy(buf, &inactiveBuf_[startChannel], len);
        retval = len;
      }
      packetSize_ = 0;
    }
  //}
  enableIRQs();
  return retval;
}

void Receiver::completePacket() {
  inPacket_ = false;

  // An empty packet isn't valid
  if (activeBufIndex_ <= 0) {
    return;
  }

  // Swap the buffers
  if (activeBuf_ == buf1_) {
    activeBuf_ = buf2_;
    inactiveBuf_ = buf1_;
  } else {
    activeBuf_ = buf1_;
    inactiveBuf_ = buf2_;
  }

  packetCount_++;
  packetSize_ = activeBufIndex_;
  packetTimestamp_ = millis();

  activeBufIndex_ = 0;
}

void Receiver::receiveBreak() {
  // Assume the actual BREAK start time is the current time, even though
  // the current time is at the end or middle of the BREAK, because the
  // difference is much smaller than a millisecond.
  // A BREAK is detected when a stop bit is expected but not received, and
  // this happens after the start bit and eight bits, about 36us.
  lastBreakTime_ = millis();

  if (inPacket_) {
    // Complete any un-flushed bytes
    // The timing can't be incorrect because, technically, the packet ended
    // with the last byte, even if it's a short packet
    completePacket();
    // TODO: Figure out how to implement a timeout, so that a short packet isn't only processed when there's the next BREAK
  } else {
    inPacket_ = true;
  }
}

void Receiver::receiveByte(uint8_t b) {
  // Ignore any extra bytes in a packet or any bytes outside a packet
  if (!inPacket_) {
    return;
  }

  // Check the timing and if we are out of range then complete any bytes
  // until, but not including, this one
  // See the notes in receiveBreak() regarding completing any un-flushed bytes
  if ((millis() - lastBreakTime_) > kMaxDMXPacketTime) {
    packetTimeoutCount_++;
    completePacket();
    return;
  }

  activeBuf_[activeBufIndex_++] = b;
  if (activeBufIndex_ == kMaxDMXPacketSize) {
    completePacket();
  }
}

// ---------------------------------------------------------------------------
//  IRQ management
// ---------------------------------------------------------------------------

void Receiver::disableIRQs() {
  switch (serialIndex_) {
    case 0:
      NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
      break;
    case 1:
      NVIC_DISABLE_IRQ(IRQ_UART1_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
      break;
    case 2:
      NVIC_DISABLE_IRQ(IRQ_UART2_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
      break;
#ifdef HAS_KINETISK_UART3
    case 3:
      NVIC_DISABLE_IRQ(IRQ_UART3_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART3_ERROR);
      break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
    case 4:
      NVIC_DISABLE_IRQ(IRQ_UART4_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART4_ERROR);
      break;
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
    case 5:
      NVIC_DISABLE_IRQ(IRQ_UART5_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART5_ERROR);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      NVIC_DISABLE_IRQ(IRQ_LPUART0);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }
}

void Receiver::enableIRQs() {
  switch (serialIndex_) {
    case 0:
      NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART0_ERROR);
      break;
    case 1:
      NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);
      break;
    case 2:
      NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);
      break;
#ifdef HAS_KINETISK_UART3
    case 3:
      NVIC_ENABLE_IRQ(IRQ_UART3_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART3_ERROR);
      break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
    case 4:
      NVIC_ENABLE_IRQ(IRQ_UART4_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART4_ERROR);
      break;
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
    case 5:
      NVIC_ENABLE_IRQ(IRQ_UART5_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART5_ERROR);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      NVIC_ENABLE_IRQ(IRQ_LPUART0);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }
}

// ---------------------------------------------------------------------------
//  UART0 RX ISR's
// ---------------------------------------------------------------------------

void uart0_rx_status_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[0];

  uint8_t status = UART0_S1;

#ifdef HAS_KINETISK_UART0_FIFO
  // If the receive buffer is full or there's an idle condition
  if ((status & (UART_S1_RDRF | UART_S1_IDLE)) != 0) {
    __disable_irq();
    uint8_t avail = UART0_RCFIFO;
    if (avail == 0) {
      // Read the register to clear the interrupt, but since it's empty,
      // this causes the FIFO to become misaligned, so send RXFLUSH to
      // reinitialize its pointers.
      // Do this inside no interrupts to avoid a potential race condition
      // between reading RCFIFO and flushing the FIFO.
      b = UART0_D;
      UART0_CFIFO = UART_CFIFO_RXFLUSH;
      __enable_irq();
      return;
    } else {
      __enable_irq();
      // Read all but the last available, then read S1 and the final value
      // So says the chip docs,
      // Section 47.3.5 UART Status Register 1 (UART_S1)
      // In the NOTE part.
      while (--avail > 0) {
        b = UART0_D;
        instance->receiveByte(b);
      }
      status = UART0_S1;
      b = UART0_D;
      instance->receiveByte(b);
    }
  }
#else
  // If the receive buffer is full
  if ((status & UART_S1_RDRF) != 0) {
    b = UART0_D;
    instance->receiveByte(b);
  }
#endif  // HAS_KINETISK_UART0_FIFO
}

void uart0_rx_error_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[0];

  // A framing error likely indicates a break
  if ((UART0_S1 & UART_S1_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a break.
    // A value of zero indicates a true break and not some other
    // framing error.
    // Note: Reading a byte clears interrupt flags

#ifdef HAS_KINETISK_UART0_FIFO
    // Flush anything in the buffer
    uint8_t avail = UART0_RCFIFO;
    if (avail > 1) {
      while (--avail > 0) {
        b = UART0_D;
        instance->receiveByte(b);
      }
    }
#endif  // HAS_KINETISK_UART0_FIFO

    b = UART0_D;
    if (b == 0) {
      instance->receiveBreak();
    } else {
      // Not a break
      instance->framingErrorCount_++;
      instance->completePacket();
    }
  }
}

// ---------------------------------------------------------------------------
//  UART1 RX ISR's
// ---------------------------------------------------------------------------

void uart1_rx_status_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[1];

  uint8_t status = UART1_S1;

#ifdef HAS_KINETISK_UART1_FIFO
  // If the receive buffer is full or there's an idle condition
  if ((status & (UART_S1_RDRF | UART_S1_IDLE)) != 0) {
    __disable_irq();
    uint8_t avail = UART1_RCFIFO;
    if (avail == 0) {
      // Read the register to clear the interrupt, but since it's empty,
      // this causes the FIFO to become misaligned, so send RXFLUSH to
      // reinitialize its pointers.
      // Do this inside no interrupts to avoid a potential race condition
      // between reading RCFIFO and flushing the FIFO.
      b = UART1_D;
      UART1_CFIFO = UART_CFIFO_RXFLUSH;
      __enable_irq();
      return;
    } else {
      __enable_irq();
      // Read all but the last available, then read S1 and the final value
      // So says the chip docs,
      // Section 47.3.5 UART Status Register 1 (UART_S1)
      // In the NOTE part.
      while (--avail > 0) {
        b = UART1_D;
        instance->receiveByte(b);
      }
      status = UART1_S1;
      b = UART1_D;
      instance->receiveByte(b);
    }
  }
#else
  // If the receive buffer is full
  if ((status & UART_S1_RDRF) != 0) {
    b = UART1_D;
    instance->receiveByte(b);
  }
#endif  // HAS_KINETISK_UART1_FIFO
}

void uart1_rx_error_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[1];

  // A framing error likely indicates a break
  if ((UART1_S1 & UART_S1_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a break.
    // A value of zero indicates a true break and not some other
    // framing error.
    // Note: Reading a byte clears interrupt flags

#ifdef HAS_KINETISK_UART1_FIFO
    // Flush anything in the buffer
    uint8_t avail = UART1_RCFIFO;
    if (avail > 1) {
      while (--avail > 0) {
        b = UART1_D;
        instance->receiveByte(b);
      }
    }
#endif  // HAS_KINETISK_UART1_FIFO

    b = UART1_D;
    if (b == 0) {
      instance->receiveBreak();
    } else {
      // Not a break
      instance->framingErrorCount_++;
      instance->completePacket();
    }
  }
}

// ---------------------------------------------------------------------------
//  UART2 RX ISR's
// ---------------------------------------------------------------------------

void uart2_rx_status_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[2];

  uint8_t status = UART2_S1;

#ifdef HAS_KINETISK_UART2_FIFO
  // If the receive buffer is full or there's an idle condition
  if ((status & (UART_S1_RDRF | UART_S1_IDLE)) != 0) {
    __disable_irq();
    uint8_t avail = UART2_RCFIFO;
    if (avail == 0) {
      // Read the register to clear the interrupt, but since it's empty,
      // this causes the FIFO to become misaligned, so send RXFLUSH to
      // reinitialize its pointers.
      // Do this inside no interrupts to avoid a potential race condition
      // between reading RCFIFO and flushing the FIFO.
      b = UART2_D;
      UART2_CFIFO = UART_CFIFO_RXFLUSH;
      __enable_irq();
      return;
    } else {
      __enable_irq();
      // Read all but the last available, then read S1 and the final value
      // So says the chip docs,
      // Section 47.3.5 UART Status Register 1 (UART_S1)
      // In the NOTE part.
      while (--avail > 0) {
        b = UART2_D;
        instance->receiveByte(b);
      }
      status = UART2_S1;
      b = UART2_D;
      instance->receiveByte(b);
    }
  }
#else
  // If the receive buffer is full
  if ((status & UART_S1_RDRF) != 0) {
    b = UART2_D;
    instance->receiveByte(b);
  }
#endif  // HAS_KINETISK_UART2_FIFO
}

void uart2_rx_error_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[2];

  // A framing error likely indicates a break
  if ((UART2_S1 & UART_S1_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a break.
    // A value of zero indicates a true break and not some other
    // framing error.
    // Note: Reading a byte clears interrupt flags

#ifdef HAS_KINETISK_UART2_FIFO
    // Flush anything in the buffer
    uint8_t avail = UART2_RCFIFO;
    if (avail > 1) {
      while (--avail > 0) {
        b = UART2_D;
        instance->receiveByte(b);
      }
    }
#endif  // HAS_KINETISK_UART2_FIFO

    b = UART2_D;
    if (b == 0) {
      instance->receiveBreak();
    } else {
      // Not a break
      instance->framingErrorCount_++;
      instance->completePacket();
    }
  }
}

// ---------------------------------------------------------------------------
//  UART3 RX ISR's
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART3
void uart3_rx_status_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[3];

  uint8_t status = UART3_S1;

  // No FIFO

  // If the receive buffer is full
  if ((status & UART_S1_RDRF) != 0) {
    b = UART3_D;
    instance->receiveByte(b);
  }
}

void uart3_rx_error_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[3];

  // A framing error likely indicates a break
  if ((UART3_S1 & UART_S1_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a break.
    // A value of zero indicates a true break and not some other
    // framing error.
    // Note: Reading a byte clears interrupt flags

    // No FIFO

    b = UART3_D;
    if (b == 0) {
      instance->receiveBreak();
    } else {
      // Not a break
      instance->framingErrorCount_++;
      instance->completePacket();
    }
  }
}
#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 RX ISR's
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART4
void uart4_rx_status_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[4];

  uint8_t status = UART4_S1;

  // No FIFO

  // If the receive buffer is full
  if ((status & UART_S1_RDRF) != 0) {
    b = UART4_D;
    instance->receiveByte(b);
  }
}

void uart4_rx_error_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[4];

  // A framing error likely indicates a break
  if ((UART4_S1 & UART_S1_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a break.
    // A value of zero indicates a true break and not some other
    // framing error.
    // Note: Reading a byte clears interrupt flags

    // No FIFO

    b = UART4_D;
    if (b == 0) {
      instance->receiveBreak();
    } else {
      // Not a break
      instance->framingErrorCount_++;
      instance->completePacket();
    }
  }
}
#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 RX ISR's
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART5
void uart5_rx_status_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[5];

  uint8_t status = UART5_S1;

  // No FIFO

  // If the receive buffer is full
  if ((status & UART_S1_RDRF) != 0) {
    b = UART5_D;
    instance->receiveByte(b);
  }
}

void uart5_rx_error_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[5];

  // A framing error likely indicates a break
  if ((UART5_S1 & UART_S1_FE) != 0) {
    // Only allow a packet whose framing error actually indicates a break.
    // A value of zero indicates a true break and not some other
    // framing error.
    // Note: Reading a byte clears interrupt flags

    // No FIFO

    b = UART5_D;
    if (b == 0) {
      instance->receiveBreak();
    } else {
      // Not a break
      instance->framingErrorCount_++;
      instance->completePacket();
    }
  }
}
#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 RX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_LPUART0
void lpuart0_rx_isr() {
  uint8_t b;
  Receiver *instance = rxInstances[5];

  uint32_t status = LPUART0_STAT;

  // No FIFO

  // If the receive buffer is full
  if ((status & LPUART_STAT_RDRF) != 0) {
    b = LPUART0_DATA;
    instance->receiveByte(b);
  }

  // A framing error likely indicates a break
  if ((status & LPUART_STAT_FE) != 0) {
    LPUART0_STAT |= LPUART_STAT_FE;  // Clear the status

    // Only allow a packet whose framing error actually indicates a break.
    // A value of zero indicates a true break and not some other
    // framing error.

    // No FIFO

    b = LPUART0_DATA;
    if (b == 0) {
      instance->receiveBreak();
    } else {
      // Not a break
      instance->framingErrorCount_++;
      instance->completePacket();
    }
  }
}
#endif  // HAS_KINETISK_LPUART0

}  // namespace teensydmx
}  // namespace qindesign
