#include "TeensyDMX.h"

// Used by the UART error ISR's.
static ::qindesign::teensydmx::TeensyDMX *instances[3]{nullptr};

#undef INSTANCE_COUNT

namespace qindesign {
namespace teensydmx {

TeensyDMX::TeensyDMX(HardwareSerial &uart)
    : uart_(uart),
      began_(false),
      buf1_{0},
      buf2_{0},
      activeBuf_(buf1_),
      inactiveBuf_(buf2_),
      activeBufIndex_(0),
      packetCount_(0),
      packetSize_(0),
      first_(true) {
  int index = -1;
  if (&uart == &Serial1) {
    index = 0;
  } else if (&uart == &Serial2) {
    index = 1;
  } else if (&uart == &Serial3) {
    index = 2;
  }
  if (index >= 0) {
    if (instances[index] != nullptr && instances[index]->began_) {
      // Even if it's the same serial port, maybe some settings have
      // changed, so end things on the current one, just in case
      instances[index]->end();
    }
    instances[index] = this;
  }
}

void TeensyDMX::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  uart_.begin(250000);

  if (&uart_ == &Serial1) {
    attachInterruptVector(IRQ_UART0_STATUS, uart0_status_isr);
    attachInterruptVector(IRQ_UART0_ERROR, uart0_error_isr);

    // // Set error IRQ priority lower than that of the status IRQ,
    // // so that the status IRQ receives any leftover bytes before
    // // we detect and trigger a new packet.
    // NVIC_SET_PRIORITY(IRQ_UART0_ERROR,
    //                   NVIC_GET_PRIORITY(IRQ_UART0_STATUS) + 1);

    // Enable UART0 interrupt on frame error
    UART0_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART0_ERROR);
  } else if (&uart_ == &Serial2) {
    attachInterruptVector(IRQ_UART1_STATUS, uart1_status_isr);
    attachInterruptVector(IRQ_UART1_ERROR, uart1_error_isr);
    // NVIC_SET_PRIORITY(IRQ_UART1_ERROR,
    //                   NVIC_GET_PRIORITY(IRQ_UART1_STATUS) + 1);
    UART1_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART1_ERROR);
  } else if (&uart_ == &Serial3) {
    attachInterruptVector(IRQ_UART2_STATUS, uart2_status_isr);
    attachInterruptVector(IRQ_UART2_ERROR, uart2_error_isr);
    // NVIC_SET_PRIORITY(IRQ_UART2_ERROR,
    //                   NVIC_GET_PRIORITY(IRQ_UART2_STATUS) + 1);
    UART2_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);
  }

  activeBuf_ = buf1_;
  inactiveBuf_ = buf2_;
}

void TeensyDMX::end() {
  if (!began_) {
    return;
  }
  began_ = false;

  uart_.end();

  if (&uart_ == &Serial1) {
    // Enable UART0 interrupt on frame error
    UART0_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
  } else if (&uart_ == &Serial2) {
    UART1_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
  } else if (&uart_ == &Serial3) {
    UART2_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
  }
}

int TeensyDMX::readPacket(uint8_t *buf) {
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

void TeensyDMX::completePacket() {
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

void TeensyDMX::resetPacket() {
  activeBufIndex_ = 0;
}

void TeensyDMX::receiveByte(uint8_t b) {
  if (activeBufIndex_ < kMaxDMXPacketSize) {
    activeBuf_[activeBufIndex_++] = b;
  }
}

void uart0_status_isr() {
  uint8_t status = UART0_S1;

  uint8_t b;
  uint8_t control;

#ifdef HAS_KINETISK_UART0_FIFO
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
        instances[0]->receiveByte(b);
      }
      status = UART0_S1;
      b = UART0_D;
      instances[0]->receiveByte(b);
    }
  }

  control = UART0_C2;
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    // Loop to send data

    // Completing if no more data
    if (status & UART_S1_TDRE) {
      UART0_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE | UART_C2_TCIE;
    }
  }
#else
  if ((status & UART_S1_RDRF) != 0) {
    b = UART0_D;
    instances[0]->receiveByte(b);
  }

  control = UART0_C2;
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    // Completing if no more data
    UART0_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_TCIE;
  }
#endif

  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    // Set inactive
    UART0_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;
  }
}

void uart1_status_isr() {
  uint8_t status = UART1_S1;

  uint8_t b;
  uint8_t control;

#ifdef HAS_KINETISK_UART1_FIFO
  if ((status & (UART_S1_RDRF | UART_S1_IDLE)) != 0) {
    __disable_irq();
    uint8_t avail = UART1_RCFIFO;
    if (avail == 0) {
      b = UART1_D;
      UART1_CFIFO = UART_CFIFO_RXFLUSH;
      __enable_irq();
      return;
    } else {
      __enable_irq();
      while (--avail > 0) {
        b = UART1_D;
        instances[1]->receiveByte(b);
      }
      status = UART1_S1;
      b = UART1_D;
      instances[1]->receiveByte(b);
    }
  }

  control = UART1_C2;
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    // Loop to send data

    // Completing if no more data
    if (status & UART_S1_TDRE) {
      UART1_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE | UART_C2_TCIE;
    }
  }
#else
  if ((status & UART_S1_RDRF) != 0) {
    b = UART1_D;
    instances[1]->receiveByte(b);
  }

  control = UART1_C2;
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    // Completing if no more data
    UART1_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_TCIE;
  }
#endif

  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    // Set inactive
    UART1_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;
  }
}

void uart2_status_isr() {
  uint8_t status = UART2_S1;

  uint8_t b;
  uint8_t control;

#ifdef HAS_KINETISK_UART2_FIFO
  if ((status & (UART_S1_RDRF | UART_S1_IDLE)) != 0) {
    __disable_irq();
    uint8_t avail = UART2_RCFIFO;
    if (avail == 0) {
      b = UART2_D;
      UART2_CFIFO = UART_CFIFO_RXFLUSH;
      __enable_irq();
      return;
    } else {
      __enable_irq();
      while (--avail > 0) {
        b = UART2_D;
        instances[2]->receiveByte(b);
      }
      status = UART2_S1;
      b = UART2_D;
      instances[2]->receiveByte(b);
    }
  }

  control = UART2_C2;
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    // Loop to send data

    // Completing if no more data
    if (status & UART_S1_TDRE) {
      UART2_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_ILIE | UART_C2_TCIE;
    }
  }
#else
  if ((status & UART_S1_RDRF) != 0) {
    b = UART2_D;
    instances[2]->receiveByte(b);
  }

  control = UART2_C2;
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    // Completing if no more data
    UART2_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE | UART_C2_TCIE;
  }
#endif

  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    // Set inactive
    UART2_C2 = UART_C2_TE | UART_C2_RE | UART_C2_RIE;
  }
}

void uart0_error_isr() {
  uint8_t b;

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
        instances[0]->receiveByte(b);
      }
    }
#endif

    b = UART0_D;
    if (b == 0) {
      instances[0]->completePacket();
    } else {
      // Not a break
      instances[0]->resetPacket();
    }
  }
}

void uart1_error_isr() {
  uint8_t b;

  if ((UART1_S1 & UART_S1_FE) != 0) {
#ifdef HAS_KINETISK_UART1_FIFO
    uint8_t avail = UART1_RCFIFO;
    if (avail > 1) {
      while (--avail > 0) {
        b = UART1_D;
        instances[1]->receiveByte(b);
      }
    }
#endif

    b = UART1_D;
    if (b == 0) {
      instances[1]->completePacket();
    } else {
      instances[1]->resetPacket();
    }
  }
}

void uart2_error_isr() {
  uint8_t b;

  if ((UART2_S1 & UART_S1_FE) != 0) {
#ifdef HAS_KINETISK_UART2_FIFO
    uint8_t avail = UART2_RCFIFO;
    if (avail > 1) {
      while (--avail > 0) {
        b = UART2_D;
        instances[2]->receiveByte(b);
      }
    }
#endif

    b = UART2_D;
    if (b == 0) {
      instances[2]->completePacket();
    } else {
      instances[2]->resetPacket();
    }
  }
}

}  // namespace teensydmx
}  // namespace qindesign
