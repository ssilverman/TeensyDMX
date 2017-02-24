#include "TeensyDMX.h"

#if defined(HAS_KINETISK_UART3)
#pragma message "Has UART3"
#define INSTANCE_COUNT 4
#elif defined(HAS_KINETISK_UART4)
#pragma message "Has UART4"
#define INSTANCE_COUNT 5
#elif defined(HAS_KINETISK_UART5)
#pragma message "Has UART5"
#define INSTANCE_COUNT 6
#else
#define INSTANCE_COUNT 3
#endif

// Used by the UART error ISR's.
static ::qindesign::teensydmx::TeensyDMX *instances[INSTANCE_COUNT] = {nullptr};

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
      bufIndex_(0),
      packetCount_(0),
      packetAvail_(false),
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
#ifdef HAS_KINETISK_UART3
  else if (&uart == &Serial4) {
    index = 3;
  }
#endif
#ifdef HAS_KINETISK_UART3
  else if (&uart == &Serial5) {
    index = 4;
  }
#endif
#ifdef HAS_KINETISK_UART3
  else if (&uart == &Serial6) {
    index = 5;
  }
#endif
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
    // Fire UART0 receive interrupt immediately after each byte received
    oldRWFIFO_ = UART0_RWFIFO;
    UART0_RWFIFO = 1;

    // Set error IRQ priority lower than that of the status IRQ,
    // so that the status IRQ receives any leftover bytes before
    // we detect and trigger a new packet.
    NVIC_SET_PRIORITY(IRQ_UART0_ERROR,
                      NVIC_GET_PRIORITY(IRQ_UART0_STATUS) + 1);

    // Enable UART0 interrupt on frame error and enable IRQ
    UART0_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART0_ERROR);
  } else if (&uart_ == &Serial2) {
    oldRWFIFO_ = UART1_RWFIFO;
    UART1_RWFIFO = 1;
    NVIC_SET_PRIORITY(IRQ_UART1_ERROR,
                      NVIC_GET_PRIORITY(IRQ_UART1_STATUS) + 1);
    UART1_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART1_ERROR);
  } else if (&uart_ == &Serial3) {
    oldRWFIFO_ = UART2_RWFIFO;
    UART2_RWFIFO = 1;
    NVIC_SET_PRIORITY(IRQ_UART2_ERROR,
                      NVIC_GET_PRIORITY(IRQ_UART2_STATUS) + 1);
    UART2_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);
  }
#ifdef HAS_KINETISK_UART3
  else if (&uart_ == &Serial4) {
    oldRWFIFO_ = UART3_RWFIFO;
    UART3_RWFIFO = 1;
    NVIC_SET_PRIORITY(IRQ_UART3_ERROR,
                      NVIC_GET_PRIORITY(IRQ_UART3_STATUS) + 1);
    UART3_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART3_ERROR);
  }
#endif
#ifdef HAS_KINETISK_UART4
  else if (&uart_ == &Serial5) {
    oldRWFIFO_ = UART4_RWFIFO;
    UART4_RWFIFO = 1;
    NVIC_SET_PRIORITY(IRQ_UART4_ERROR,
                      NVIC_GET_PRIORITY(IRQ_UART4_STATUS) + 1);
    UART4_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART4_ERROR);
  }
#endif
#ifdef HAS_KINETISK_UART5
  else if (&uart_ == &Serial6) {
    oldRWFIFO_ = UART5_RWFIFO;
    UART5_RWFIFO = 1;
    NVIC_SET_PRIORITY(IRQ_UART5_ERROR,
                      NVIC_GET_PRIORITY(IRQ_UART5_STATUS) + 1);
    UART5_C3 |= UART_C3_FEIE;
    NVIC_ENABLE_IRQ(IRQ_UART5_ERROR);
  }
#endif

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
    UART0_RWFIFO = oldRWFIFO_;
    UART0_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
  } else if (&uart_ == &Serial2) {
    UART1_RWFIFO = oldRWFIFO_;
    UART1_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
  } else if (&uart_ == &Serial3) {
    UART2_RWFIFO = oldRWFIFO_;
    UART2_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
  }
#ifdef HAS_KINETISK_UART3
  else if (&uart_ == &Serial4) {
    UART3_RWFIFO = oldRWFIFO_;
    UART3_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART3_ERROR);
  }
#endif
#ifdef HAS_KINETISK_UART4
  else if (&uart_ == &Serial5) {
    UART4_RWFIFO = oldRWFIFO_;
    UART4_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART4_ERROR);
  }
#endif
#ifdef HAS_KINETISK_UART5
  else if (&uart_ == &Serial6) {
    UART5_RWFIFO = oldRWFIFO_;
    UART5_C3 &= static_cast<uint8_t>(~UART_C3_FEIE);
    NVIC_DISABLE_IRQ(IRQ_UART5_ERROR);
  }
#endif
}

bool TeensyDMX::packetAvailable() {
  bool retval = false;
  __disable_irq();
  //{
    if (packetAvail_) {
      packetAvail_ = false;
      retval = true;
    }
  //}
  __enable_irq();
  return retval;
}

int TeensyDMX::readPacket(uint8_t *buf) {
  int retval = -1;
  __disable_irq();
  //{
    if (packetAvail_) {
      packetAvail_ = false;
      memcpy(buf, const_cast<const uint8_t*>(inactiveBuf_), 513);
      retval = 513;
      // TODO(shawn): Why is packetSize_ always zero??
    }
  //}
  __enable_irq();
  return retval;
}

void TeensyDMX::completePacket() {
  // 1. Fill the buffer

  __disable_irq();  // Prevents conflicts with the UART0 status ISR
  //{
    int avail = uart_.available();

    if (!first_) {
      int count = min(avail, kMaxDMXPacketSize - bufIndex_);
      while (count-- > 0) {
        activeBuf_[bufIndex_++] = static_cast<uint8_t>(uart_.read());
        avail--;
      }
    }

    // Flush any remaining bytes
    while (avail > 0) {
      uart_.read();
      avail--;
    }
  //}
  __enable_irq();

  // 2. Complete the packet

  if (first_) {
    first_ = false;
    return;
  }

  // Reset the buffers
  if (activeBuf_ == buf1_) {
    activeBuf_ = buf2_;
    inactiveBuf_ = buf1_;
  } else {
    activeBuf_ = buf1_;
    inactiveBuf_ = buf2_;
  }

  packetSize_ = bufIndex_;
  bufIndex_ = 0;

  packetCount_++;
  packetAvail_ = true;
}

// TODO(shawn): Is flushing the input the right approach for framing errors that aren't a break?
void TeensyDMX::flushInput() const {
  __disable_irq();
  //{
    int avail = uart_.available();
    while (avail > 0) {
      uart_.read();
      avail--;
    }
  //}
  __enable_irq();
}

}  // namespace teensydmx
}  // namespace qindesign

// These error ISR routines need to be outside the namespace:

// UART0 will throw a frame error on the DMX break pulse. That's our
// cue to switch buffers and reset the index to zero.
void uart0_error_isr() {
  // On break, uart0_status_isr() will probably have already
  // fired and read the data buffer, clearing the framing error.
  // If for some reason it hasn't, make sure we consume the 0x00
  // byte that was received.
  uint8_t b = 0;
  if ((UART0_S1 & UART_S1_FE) != 0) {
    b = UART0_D;
  }

  // Ensure we've processed all the data that may still be sitting
  // in software buffers.
  if (b == 0) {
    instances[0]->completePacket();
  } else {
    // Not a break
    instances[0]->flushInput();
  }
}

void uart1_error_isr() {
  uint8_t b = 0;
  if ((UART1_S1 & UART_S1_FE) != 0) {
    b = UART1_D;
  }
  if (b == 0) {
    instances[1]->completePacket();
  } else {
    // Not a break
    instances[1]->flushInput();
  }
}

void uart2_error_isr() {
  uint8_t b = 0;
  if ((UART2_S1 & UART_S1_FE) != 0) {
    b = UART2_D;
  }
  if (b == 0) {
    instances[2]->completePacket();
  } else {
    // Not a break
    instances[2]->flushInput();
  }
}

#ifdef HAS_KINETISK_UART3
void uart3_error_isr() {
  uint8_t b = 0;
  if ((UART3_S1 & UART_S1_FE) != 0) {
    b = UART3_D;
  }
  if (b == 0) {
    instances[3]->completePacket();
  } else {
    // Not a break
    instances[3]->flushInput();
  }
}
#endif

#ifdef HAS_KINETISK_UART4
void uart4_error_isr() {
  uint8_t b = 0;
  if ((UART4_S1 & UART_S1_FE) != 0) {
    b = UART4_D;
  }
  if (b == 0) {
    instances[4]->completePacket();
  } else {
    // Not a break
    instances[4]->flushInput();
  }
}
#endif

#ifdef HAS_KINETISK_UART5
void uart5_error_isr() {
  uint8_t b = 0;
  if ((UART5_S1 & UART_S1_FE) != 0) {
    b = UART5_D;
  }
  if (b == 0) {
    instances[5]->completePacket();
  } else {
    // Not a break
    instances[5]->flushInput();
  }
}
#endif
