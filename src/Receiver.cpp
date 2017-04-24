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

// Used by the RX ISR's.
Receiver *rxInstances[3]{nullptr};

void Receiver::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  int index = serialIndex(uart_);
  if (index < 0) {
    return;
  }

  // Set up the instance for the ISRs
  if (rxInstances[index] != nullptr) {
    rxInstances[index]->end();
  }
  rxInstances[index] = this;

  uart_.begin(kSlotsBaud, kSlotsFormat);

  switch (index) {
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
  }

  activeBuf_ = buf1_;
  inactiveBuf_ = buf2_;
}

void Receiver::end() {
  if (!began_) {
    return;
  }
  began_ = false;

  int index = serialIndex(uart_);
  if (index < 0) {
    return;
  }

  // Remove the reference from the instances
  rxInstances[index] = nullptr;

  uart_.end();

  switch (index) {
    case 0:
      // Disable UART0 interrupt on frame error
      UART0_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
    case 1:
      UART1_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
    case 2:
      UART2_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
  }
}

int Receiver::readPacket(uint8_t *buf, int startChannel, int len) {
  if (packetSize_ <= 0) {
    return -1;
  }
  if (len <= 0 || startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return 0;
  }

  int retval = -1;
  __disable_irq();
  //{
    if (packetSize_ > 0) {
      if (startChannel >= packetSize_) {
        retval = 0;
      } else {
        if (startChannel + len > packetSize_) {
          len = packetSize_ - startChannel;
        }
        memcpy(
            buf,
            const_cast<const uint8_t*>(inactiveBuf_ + startChannel),
            len);
        retval = len;
      }
      packetSize_ = 0;
    }
  //}
  __enable_irq();
  return retval;
}

void Receiver::completePacket() {
  // An empty packet isn't valid
  if (activeBufIndex_ <= 0) {
    return;
  }

  if (first_) {
    first_ = false;
  } else {
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
  }
  activeBufIndex_ = 0;
}

void Receiver::resetPacket() {
  activeBufIndex_ = 0;
}

void Receiver::receiveByte(uint8_t b) {
  if (activeBufIndex_ < kMaxDMXPacketSize) {
    activeBuf_[activeBufIndex_++] = b;
  }
}

// ---------------------------------------------------------------------------
//  UART0 RX ISRs
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

  // A framing error indicates a break
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
      instance->completePacket();
    } else {
      // Not a break
      instance->resetPacket();
    }
  }
}

// ---------------------------------------------------------------------------
//  UART1 RX ISRs
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

  // A framing error indicates a break
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
      instance->completePacket();
    } else {
      // Not a break
      instance->resetPacket();
    }
  }
}

// ---------------------------------------------------------------------------
//  UART2 RX ISRs
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

  // A framing error indicates a break
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
      instance->completePacket();
    } else {
      // Not a break
      instance->resetPacket();
    }
  }
}

}  // namespace teensydmx
}  // namespace qindesign
