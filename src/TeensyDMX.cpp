#include "TeensyDMX.h"

// C++ includes
#include <cstring>

namespace qindesign {
namespace teensydmx {

// Notes on transmit timing:
// According to https://en.wikipedia.org/wiki/DMX512,
// the minimum break and Mark AfterBreak (MAB) times are
// 92us and 12us, respectively.
//
// If we assume 12us is the length of a stop bit, then 1/12us ≈ 83333 baud.
// For 8N1, the length of the 9 bits before the stop bit ≈ 108us.
//
// Minimum accepted receive break-to-break time = 1196us.
// This means that we must transmit at least 24 slots (25 including the
// start code).

constexpr uint32_t kBreakBaud = 1000000 / 12;
constexpr uint32_t kBreakFormat = SERIAL_8N1;
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

// TX control states
#define UART_C2_TX_ENABLE     UART_C2_TE
#define UART_C2_TX_ACTIVE     UART_C2_TX_ENABLE | UART_C2_TIE
#define UART_C2_TX_COMPLETING UART_C2_TX_ENABLE | UART_C2_TCIE
#define UART_C2_TX_INACTIVE   UART_C2_TX_ENABLE

// ---------------------------------------------------------------------------
//  TeensyDMXReceiver
// ---------------------------------------------------------------------------

// Used by the TX ISR's.
static TeensyDMXReceiver *rxInstances[3]{nullptr};

void TeensyDMXReceiver::begin() {
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

void TeensyDMXReceiver::end() {
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

int TeensyDMXReceiver::readPacket(uint8_t *buf) {
  if (packetSize_ <= 0) {
    return -1;
  }

  int retval = -1;
  __disable_irq();
  //{
    if (packetSize_ > 0) {
      memcpy(buf, const_cast<const uint8_t*>(inactiveBuf_), packetSize_);
      retval = packetSize_;
      packetSize_ = 0;
    }
  //}
  __enable_irq();
  return retval;
}

void TeensyDMXReceiver::completePacket() {
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
  }
  activeBufIndex_ = 0;
}

void TeensyDMXReceiver::resetPacket() {
  activeBufIndex_ = 0;
}

void TeensyDMXReceiver::receiveByte(uint8_t b) {
  if (activeBufIndex_ < kMaxDMXPacketSize) {
    activeBuf_[activeBufIndex_++] = b;
  }
}

// ---------------------------------------------------------------------------
//  UART0 RX ISRs
// ---------------------------------------------------------------------------

void uart0_rx_status_isr() {
  uint8_t b;
  TeensyDMXReceiver *instance = static_cast<TeensyDMXReceiver*>(rxInstances[0]);

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
  TeensyDMXReceiver *instance = static_cast<TeensyDMXReceiver*>(rxInstances[0]);

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
  TeensyDMXReceiver *instance = static_cast<TeensyDMXReceiver*>(rxInstances[1]);

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
  TeensyDMXReceiver *instance = static_cast<TeensyDMXReceiver*>(rxInstances[1]);

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
  TeensyDMXReceiver *instance = static_cast<TeensyDMXReceiver*>(rxInstances[2]);

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
  TeensyDMXReceiver *instance = static_cast<TeensyDMXReceiver*>(rxInstances[2]);

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

// ---------------------------------------------------------------------------
//  TeensyDMXSender
// ---------------------------------------------------------------------------

// Used by the TX ISR's.
static TeensyDMXSender *txInstances[3]{nullptr};

void TeensyDMXSender::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  int index = serialIndex(uart_);
  if (index < 0) {
    return;
  }

  // Set up the instance for the ISRs
  if (txInstances[index] != nullptr) {
    txInstances[index]->end();
  }
  txInstances[index] = this;

  state_ = XmitStates::kBreak;
  uart_.begin(kBreakBaud, kBreakFormat);

  switch (index) {
    case 0:
      attachInterruptVector(IRQ_UART0_STATUS, uart0_tx_status_isr);
      UART0_C2 = UART_C2_TX_ACTIVE;
      break;
    case 1:
      attachInterruptVector(IRQ_UART1_STATUS, uart1_tx_status_isr);
      UART1_C2 = UART_C2_TX_ACTIVE;
      break;
    case 2:
      attachInterruptVector(IRQ_UART2_STATUS, uart2_tx_status_isr);
      UART2_C2 = UART_C2_TX_ACTIVE;
      break;
  }
}

void TeensyDMXSender::end() {
  // Remove any chance that our TX ISR calls begin after end() is called
  NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);

  if (!began_) {
    return;
  }
  began_ = false;

  int index = serialIndex(uart_);
  if (index < 0) {
    return;
  }

  // Remove the reference from the instances
  txInstances[index] = nullptr;

  uart_.end();
}

void TeensyDMXSender::set(int startChannel, const uint8_t *values, int len) {
  if (len <= 0) {
    return;
  }
  if (startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return;
  }
  if (startChannel + len <= 0 || kMaxDMXPacketSize < startChannel + len) {
    return;
  }

  memcpy(const_cast<uint8_t*>(outputBuf_ + startChannel), values, len);
}

void TeensyDMXSender::completePacket() {
  packetCount_++;
  outputBufIndex_ = 0;
  state_ = XmitStates::kIdle;
}

// ---------------------------------------------------------------------------
//  UART0 TX ISR
// ---------------------------------------------------------------------------

void uart0_tx_status_isr() {
  TeensyDMXSender *instance = static_cast<TeensyDMXSender*>(txInstances[0]);

  uint8_t status = UART0_S1;
  uint8_t control = UART0_C2;

#ifdef HAS_KINETISK_UART0_FIFO
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kBreak:
        UART0_D = 0;
        UART0_C2 = UART_C2_TX_COMPLETING;
        break;

      case TeensyDMXSender::XmitStates::kData:
        do {
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART0_C2 = UART_C2_TX_COMPLETING;
            break;
          }
          status = UART0_S1;
          UART0_D = instance->outputBuf_[instance->outputBufIndex_++];
        } while (UART0_TCFIFO < 8);

        break;

      case TeensyDMXSender::XmitStates::kIdle:
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kBreak:
        UART0_D = 0;
        UART0_C2 = UART_C2_TX_COMPLETING;
        break;

      case TeensyDMXSender::XmitStates::kData:
        if (instance->outputBufIndex_ >= instance->packetSize_) {
          instance->completePacket();
          UART0_C2 = UART_C2_TX_COMPLETING;
        } else {
          UART0_D = instance->outputBuf_[instance->outputBufIndex_++];
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART0_C2 = UART_C2_TX_COMPLETING;
          }
        }
        break;

      case TeensyDMXSender::XmitStates::kIdle:
        break;
    }
  }
#endif  // HAS_KINETISK_UART0_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kIdle:
        instance->state_ = TeensyDMXSender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);
        break;

      case TeensyDMXSender::XmitStates::kBreak:
        instance->state_ = TeensyDMXSender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case TeensyDMXSender::XmitStates::kData:
        break;
    }
    UART0_C2 = UART_C2_TX_ACTIVE;
  }
}

// ---------------------------------------------------------------------------
//  UART1 TX ISR
// ---------------------------------------------------------------------------

void uart1_tx_status_isr() {
  TeensyDMXSender *instance = static_cast<TeensyDMXSender*>(txInstances[1]);

  uint8_t status = UART1_S1;
  uint8_t control = UART1_C2;

#ifdef HAS_KINETISK_UART1_FIFO
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kBreak:
        UART1_D = 0;
        UART1_C2 = UART_C2_TX_COMPLETING;
        break;

      case TeensyDMXSender::XmitStates::kData:
        do {
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART1_C2 = UART_C2_TX_COMPLETING;
            break;
          }
          status = UART1_S1;
          UART1_D = instance->outputBuf_[instance->outputBufIndex_++];
        } while (UART1_TCFIFO < 8);

        break;

      case TeensyDMXSender::XmitStates::kIdle:
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kBreak:
        UART1_D = 0;
        UART1_C2 = UART_C2_TX_COMPLETING;
        break;

      case TeensyDMXSender::XmitStates::kData:
        if (instance->outputBufIndex_ >= instance->packetSize_) {
          instance->completePacket();
          UART1_C2 = UART_C2_TX_COMPLETING;
        } else {
          UART1_D = instance->outputBuf_[instance->outputBufIndex_++];
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART1_C2 = UART_C2_TX_COMPLETING;
          }
        }
        break;

      case TeensyDMXSender::XmitStates::kIdle:
        break;
    }
  }
#endif  // HAS_KINETISK_UART1_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kIdle:
        instance->state_ = TeensyDMXSender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);
        break;

      case TeensyDMXSender::XmitStates::kBreak:
        instance->state_ = TeensyDMXSender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case TeensyDMXSender::XmitStates::kData:
        break;
    }
    UART1_C2 = UART_C2_TX_ACTIVE;
  }
}

// ---------------------------------------------------------------------------
//  UART2 TX ISR
// ---------------------------------------------------------------------------

void uart2_tx_status_isr() {
  TeensyDMXSender *instance = static_cast<TeensyDMXSender*>(txInstances[2]);

  uint8_t status = UART2_S1;
  uint8_t control = UART2_C2;

#ifdef HAS_KINETISK_UART2_FIFO
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kBreak:
        UART2_D = 0;
        UART2_C2 = UART_C2_TX_COMPLETING;
        break;

      case TeensyDMXSender::XmitStates::kData:
        do {
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART2_C2 = UART_C2_TX_COMPLETING;
            break;
          }
          status = UART2_S1;
          UART2_D = instance->outputBuf_[instance->outputBufIndex_++];
        } while (UART2_TCFIFO < 8);

        break;

      case TeensyDMXSender::XmitStates::kIdle:
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kBreak:
        UART2_D = 0;
        UART2_C2 = UART_C2_TX_COMPLETING;
        break;

      case TeensyDMXSender::XmitStates::kData:
        if (instance->outputBufIndex_ >= instance->packetSize_) {
          instance->completePacket();
          UART2_C2 = UART_C2_TX_COMPLETING;
        } else {
          UART2_D = instance->outputBuf_[instance->outputBufIndex_++];
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART2_C2 = UART_C2_TX_COMPLETING;
          }
        }
        break;

      case TeensyDMXSender::XmitStates::kIdle:
        break;
    }
  }
#endif  // HAS_KINETISK_UART2_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case TeensyDMXSender::XmitStates::kIdle:
        instance->state_ = TeensyDMXSender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);
        break;

      case TeensyDMXSender::XmitStates::kBreak:
        instance->state_ = TeensyDMXSender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case TeensyDMXSender::XmitStates::kData:
        break;
    }
    UART2_C2 = UART_C2_TX_ACTIVE;
  }
}

}  // namespace teensydmx
}  // namespace qindesign
