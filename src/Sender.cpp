#include "TeensyDMX.h"

// C++ includes
#include <cstring>

namespace qindesign {
namespace teensydmx {

// Notes on transmit timing:
// According to https://en.wikipedia.org/wiki/DMX512,
// the minimum break and Mark After Break (MAB) times are
// 92us and 12us, respectively.
//
// If we assume 12us is the length of a stop bit, then 1/12us ≈ 83333 baud.
// For 8N1, the length of the 9 bits before the stop bit ≈ 108us.
//
// Minimum accepted receive break-to-break time = 1196us.
// This means that we must transmit at least 24 slots (25 including the
// start code).
//
// Some other timing options:
// 8N2: 1000000/11 (90909) baud, 99us break, 22us MAB
// 8E2: 100000 baud, 100us break, 20us MAB
// 8N1: 50000 baud, 180us break, 20us MAB <-- Closer to "typical" in ANSI E1.11
// 8E1: 45500 baud, 220us break, 22us MAB

constexpr uint32_t kBreakBaud   = 50000;
constexpr uint32_t kBreakFormat = SERIAL_8N1;
constexpr uint32_t kSlotsBaud   = 250000;
constexpr uint32_t kSlotsFormat = SERIAL_8N2;

// TX control states
#define UART_C2_TX_ENABLE         (UART_C2_TE)
#define UART_C2_TX_ACTIVE         ((UART_C2_TX_ENABLE) | (UART_C2_TIE))
#define UART_C2_TX_COMPLETING     ((UART_C2_TX_ENABLE) | (UART_C2_TCIE))
#define UART_C2_TX_INACTIVE       (UART_C2_TX_ENABLE)
#define LPUART_CTRL_TX_ENABLE     (LPUART_CTRL_TE)
#define LPUART_CTRL_TX_ACTIVE     ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TIE))
#define LPUART_CTRL_TX_COMPLETING ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TCIE))
#define LPUART_CTRL_TX_INACTIVE   (LPUART_CTRL_TX_ENABLE)

// Used by the TX ISR's.
Sender *txInstances[6]{nullptr};

void Sender::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  if (serialIndex_ < 0) {
    return;
  }

  // Set up the instance for the ISR's
  if (txInstances[serialIndex_] != nullptr) {
    txInstances[serialIndex_]->end();
  }
  txInstances[serialIndex_] = this;

  state_ = XmitStates::kIdle;
  uart_.begin(kBreakBaud, kBreakFormat);

  switch (serialIndex_) {
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
#ifdef HAS_KINETISK_UART3
    case 3:
      attachInterruptVector(IRQ_UART3_STATUS, uart3_tx_status_isr);
      UART3_C2 = UART_C2_TX_ACTIVE;
      break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
    case 4:
      attachInterruptVector(IRQ_UART4_STATUS, uart4_tx_status_isr);
      UART4_C2 = UART_C2_TX_ACTIVE;
      break;
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
    case 5:
      attachInterruptVector(IRQ_UART5_STATUS, uart5_tx_status_isr);
      UART5_C2 = UART_C2_TX_ACTIVE;
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      attachInterruptVector(IRQ_LPUART0, lpuart0_tx_isr);
      LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE;
      break;
#endif  // HAS_KINETISK_LPUART0 || HAS_KINETISK_UART5
  }
}

void Sender::end() {
  if (serialIndex_ < 0) {
    return;
  }

  // Remove any chance that our TX ISR calls begin after end() is called
  switch (serialIndex_) {
    case 0:
      NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
      break;
    case 1:
      NVIC_DISABLE_IRQ(IRQ_UART1_STATUS);
      break;
    case 2:
      NVIC_DISABLE_IRQ(IRQ_UART2_STATUS);
      break;
#ifdef HAS_KINETISK_UART3
    case 3:
      NVIC_DISABLE_IRQ(IRQ_UART3_STATUS);
      break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
    case 4:
      NVIC_DISABLE_IRQ(IRQ_UART4_STATUS);
      break;
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
    case 5:
      NVIC_DISABLE_IRQ(IRQ_UART5_STATUS);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      NVIC_DISABLE_IRQ(IRQ_LPUART0);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }

  if (!began_) {
    return;
  }
  began_ = false;

  // Remove the reference from the instances
  txInstances[serialIndex_] = nullptr;

  uart_.end();
}

// memcpy implementation that accepts a volatile destination.
// Derived from:
// https://github.com/ARM-software/arm-trusted-firmware/blob/master/lib/stdlib/mem.c
static volatile void *memcpy(volatile void *dst, const void *src, size_t len) {
  volatile uint8_t *d = reinterpret_cast<volatile uint8_t *>(dst);
  const uint8_t *s = reinterpret_cast<const uint8_t *>(src);

  while (len-- != 0) {
    *(d++) = *(s++);
  }

  return dst;
}

void Sender::set(int startChannel, const uint8_t *values, int len) {
  if (len <= 0) {
    return;
  }
  if (startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return;
  }
  if (startChannel + len <= 0 || kMaxDMXPacketSize < startChannel + len) {
    return;
  }

  memcpy(outputBuf_ + startChannel, values, len);
}

void Sender::setRefreshRate(float rate) {
  if (std::isnan(rate) || rate < 0.0f) {
    return;
  }
  if (rate == 0.0f) {
    breakToBreakTime_ = UINT32_MAX;
  } else {
    if (refreshRate_ == 0.0f) {
      end();
      begin();
    }
    breakToBreakTime_ = 1000000 / rate;
  }
  refreshRate_ = rate;
}

void Sender::completePacket() {
  packetCount_++;
  outputBufIndex_ = 0;
  state_ = XmitStates::kIdle;
}

// ---------------------------------------------------------------------------
//  UART0 TX ISR
// ---------------------------------------------------------------------------

void uart0_tx_status_isr() {
  Sender *instance = txInstances[0];

  uint8_t status = UART0_S1;
  uint8_t control = UART0_C2;

#ifdef HAS_KINETISK_UART0_FIFO
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART0_D = 0;
        instance->timeSinceBreak_ = 0;
        UART0_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
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

      case Sender::XmitStates::kIdle: {
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART0_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[0]->timer_.end();
                      UART0_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART0_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART0_C2 = UART_C2_TX_ACTIVE;
        }
        break;
      }
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART0_D = 0;
        instance->timeSinceBreak_ = 0;
        UART0_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
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

      case Sender::XmitStates::kIdle: {
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART0_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[0]->timer_.end();
                      UART0_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART0_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART0_C2 = UART_C2_TX_ACTIVE;
        }
        break;
      }
    }
  }
#endif  // HAS_KINETISK_UART0_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
      case Sender::XmitStates::kIdle:
        break;
    }
    UART0_C2 = UART_C2_TX_ACTIVE;
  }
}

// ---------------------------------------------------------------------------
//  UART1 TX ISR
// ---------------------------------------------------------------------------

void uart1_tx_status_isr() {
  Sender *instance = txInstances[1];

  uint8_t status = UART1_S1;
  uint8_t control = UART1_C2;

#ifdef HAS_KINETISK_UART1_FIFO
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART1_D = 0;
        instance->timeSinceBreak_ = 0;
        UART1_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
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

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART1_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[1]->timer_.end();
                      UART1_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART1_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART1_C2 = UART_C2_TX_ACTIVE;
        }
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART1_D = 0;
        instance->timeSinceBreak_ = 0;
        UART1_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
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

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART1_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[1]->timer_.end();
                      UART1_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART1_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART1_C2 = UART_C2_TX_ACTIVE;
        }
        break;
    }
  }
#endif  // HAS_KINETISK_UART1_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
      case Sender::XmitStates::kIdle:
        break;
    }
    UART1_C2 = UART_C2_TX_ACTIVE;
  }
}

// ---------------------------------------------------------------------------
//  UART2 TX ISR
// ---------------------------------------------------------------------------

void uart2_tx_status_isr() {
  Sender *instance = txInstances[2];

  uint8_t status = UART2_S1;
  uint8_t control = UART2_C2;

#ifdef HAS_KINETISK_UART2_FIFO
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART2_D = 0;
        instance->timeSinceBreak_ = 0;
        UART2_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
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

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART2_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[2]->timer_.end();
                      UART2_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART2_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART2_C2 = UART_C2_TX_ACTIVE;
        }
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART2_D = 0;
        instance->timeSinceBreak_ = 0;
        UART2_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
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

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART2_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[2]->timer_.end();
                      UART2_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART2_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART2_C2 = UART_C2_TX_ACTIVE;
        }
        break;
    }
  }
#endif  // HAS_KINETISK_UART2_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
      case Sender::XmitStates::kIdle:
        break;
    }
    UART2_C2 = UART_C2_TX_ACTIVE;
  }
}

// ---------------------------------------------------------------------------
//  UART3 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART3
void uart3_tx_status_isr() {
  Sender *instance = txInstances[3];

  uint8_t status = UART3_S1;
  uint8_t control = UART3_C2;

  // No FIFO

  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART3_D = 0;
        instance->timeSinceBreak_ = 0;
        UART3_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
        if (instance->outputBufIndex_ >= instance->packetSize_) {
          instance->completePacket();
          UART3_C2 = UART_C2_TX_COMPLETING;
        } else {
          UART3_D = instance->outputBuf_[instance->outputBufIndex_++];
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART3_C2 = UART_C2_TX_COMPLETING;
          }
        }
        break;

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART3_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[3]->timer_.end();
                      UART3_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART3_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART3_C2 = UART_C2_TX_ACTIVE;
        }
        break;
    }
  }

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
      case Sender::XmitStates::kIdle:
        break;
    }
    UART3_C2 = UART_C2_TX_ACTIVE;
  }
}
#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART4
void uart4_tx_status_isr() {
  Sender *instance = txInstances[4];

  uint8_t status = UART4_S1;
  uint8_t control = UART4_C2;

  // No FIFO

  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART4_D = 0;
        instance->timeSinceBreak_ = 0;
        UART4_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
        if (instance->outputBufIndex_ >= instance->packetSize_) {
          instance->completePacket();
          UART4_C2 = UART_C2_TX_COMPLETING;
        } else {
          UART4_D = instance->outputBuf_[instance->outputBufIndex_++];
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART4_C2 = UART_C2_TX_COMPLETING;
          }
        }
        break;

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART4_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[4]->timer_.end();
                      UART4_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART4_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART4_C2 = UART_C2_TX_ACTIVE;
        }
        break;
    }
  }

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
      case Sender::XmitStates::kIdle:
        break;
    }
    UART4_C2 = UART_C2_TX_ACTIVE;
  }
}
#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART5
void uart5_tx_status_isr() {
  Sender *instance = txInstances[5];

  uint8_t status = UART5_S1;
  uint8_t control = UART5_C2;

  // No FIFO

  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART5_D = 0;
        instance->timeSinceBreak_ = 0;
        UART5_C2 = UART_C2_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
        if (instance->outputBufIndex_ >= instance->packetSize_) {
          instance->completePacket();
          UART5_C2 = UART_C2_TX_COMPLETING;
        } else {
          UART5_D = instance->outputBuf_[instance->outputBufIndex_++];
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            UART5_C2 = UART_C2_TX_COMPLETING;
          }
        }
        break;

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          UART5_C2 = UART_C2_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[5]->timer_.end();
                      UART5_C2 = UART_C2_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              UART5_C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          UART5_C2 = UART_C2_TX_ACTIVE;
        }
        break;
    }
  }

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
      case Sender::XmitStates::kIdle:
        break;
    }
    UART5_C2 = UART_C2_TX_ACTIVE;
  }
}
#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_LPUART0
void lpuart0_tx_isr() {
  Sender *instance = txInstances[5];

  uint32_t status = LPUART0_STAT;
  uint32_t control = LPUART0_CTRL;

  // No FIFO

  // If the transmit buffer is empty
  if ((control & LPUART_CTRL_TIE) != 0 && (status & LPUART_STAT_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        LPUART0_DATA = 0;
        instance->timeSinceBreak_ = 0;
        LPUART0_CTRL = LPUART_CTRL_TX_COMPLETING;
        break;

      case Sender::XmitStates::kData:
        if (instance->outputBufIndex_ >= instance->packetSize_) {
          instance->completePacket();
          LPUART0_CTRL = LPUART_CTRL_TX_COMPLETING;
        } else {
          LPUART0_DATA = instance->outputBuf_[instance->outputBufIndex_++];
          if (instance->outputBufIndex_ >= instance->packetSize_) {
            instance->completePacket();
            LPUART0_CTRL = LPUART_CTRL_TX_COMPLETING;
          }
        }
        break;

      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          LPUART0_CTRL = LPUART_CTRL_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->timer_.begin(
                    []() {
                      txInstances[5]->timer_.end();
                      LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE;
                    },
                    instance->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE;
        }
        break;
    }
  }

  // If transmission is complete
  if ((control & LPUART_CTRL_TCIE) != 0 && (status & LPUART_STAT_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
      case Sender::XmitStates::kIdle:
        break;
    }
    LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE;
  }
}
#endif  // HAS_KINETISK_LPUART0

}  // namespace teensydmx
}  // namespace qindesign
