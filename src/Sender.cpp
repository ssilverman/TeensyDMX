#include "TeensyDMX.h"

// C++ includes
#include <cstring>

// Project includes
#include "lock_routines.h"
#include "uart_routine_defines.h"

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

static constexpr uint32_t kBreakBaud   = 50000;
static constexpr uint32_t kBreakFormat = SERIAL_8N1;
static constexpr uint32_t kSlotsBaud   = 250000;
static constexpr uint32_t kSlotsFormat = SERIAL_8N2;

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
static Sender *volatile txInstances[6]{nullptr};
static volatile bool txInstancesMutex{false};

Sender::Sender(HardwareSerial &uart)
    : TeensyDMX(uart),
      state_(XmitStates::kIdle),
      outputBuf_{0},
      outputBufIndex_(0),
      packetSize_(kMaxDMXPacketSize),
      refreshRate_(INFINITY),
      breakToBreakTime_(0),
      paused_(false),
      resumeCounter_(0),
      transmitting_(false) {}

Sender::~Sender() {
  end();
}

#define ACTIVATE_TX_SERIAL(N)\
  attachInterruptVector(IRQ_UART##N##_STATUS, uart##N##_tx_status_isr);\
  UART##N##_C2 = UART_C2_TX_ACTIVE;

void Sender::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  if (serialIndex_ < 0) {
    return;
  }

  // Set up the instance for the ISR's
  grabMutex(&txInstancesMutex);
  Sender *s = txInstances[serialIndex_];
  txInstances[serialIndex_] = this;
  releaseMutex(&txInstancesMutex);
  if (s != nullptr && s != this) {  // NOTE: Shouldn't be able to be 'this'
    s->end();
  }

  transmitting_ = false;
  state_ = XmitStates::kIdle;
  uart_.begin(kBreakBaud, kBreakFormat);

  switch (serialIndex_) {
    case 0:
      ACTIVATE_TX_SERIAL(0)
      break;
    case 1:
      ACTIVATE_TX_SERIAL(1)
      break;
    case 2:
      ACTIVATE_TX_SERIAL(2)
      break;
#ifdef HAS_KINETISK_UART3
    case 3:
      ACTIVATE_TX_SERIAL(3)
      break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
    case 4:
      ACTIVATE_TX_SERIAL(4)
      break;
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
    case 5:
      ACTIVATE_TX_SERIAL(5)
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      attachInterruptVector(IRQ_LPUART0, lpuart0_tx_isr);
      LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE;
      break;
#endif  // HAS_KINETISK_LPUART0 || HAS_KINETISK_UART5
  }
}

// Undefine this macro
#undef ACTIVATE_TX_SERIAL

void Sender::end() {
  if (!began_) {
    return;
  }
  began_ = false;

  if (serialIndex_ < 0) {
    return;
  }

  // Remove any chance that our TX ISR's start after end() is called,
  // so disable the IRQ's first

  uart_.end();

  // Remove the reference from the instances,
  // but only if we're the ones who added it
  grabMutex(&txInstancesMutex);
  if (txInstances[serialIndex_] == this) {
    txInstances[serialIndex_] = nullptr;
  }
  releaseMutex(&txInstancesMutex);
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

void Sender::resume() {
  resumeFor(0);
}

void Sender::resumeFor(int n) {
  if (n < 0) {
    return;
  }

  // Pausing made transmission INACTIVE
  __disable_irq();
  resumeCounter_ = n;
  if (paused_ && !transmitting_) {
    switch (serialIndex_) {
      case 0: UART0_C2 = UART_C2_TX_ACTIVE; break;
      case 1: UART1_C2 = UART_C2_TX_ACTIVE; break;
      case 2: UART2_C2 = UART_C2_TX_ACTIVE; break;
#ifdef HAS_KINETISK_UART3
      case 3: UART3_C2 = UART_C2_TX_ACTIVE; break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
      case 4: UART4_C2 = UART_C2_TX_ACTIVE; break;
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
      case 5: UART5_C2 = UART_C2_TX_ACTIVE; break;
#elif defined(HAS_KINETISK_LPUART0)
      case 5: LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE; break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
    }
  }
  paused_ = false;
  __enable_irq();
}

bool Sender::isTransmitting() {
  // Check these both atomically
  __disable_irq();
  bool state = !paused_ || transmitting_;
  __enable_irq();
  return state;
}

void Sender::completePacket() {
  packetCount_++;
  outputBufIndex_ = 0;
  transmitting_ = false;
  state_ = XmitStates::kIdle;
}

// ---------------------------------------------------------------------------
//  UART0 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART0_FIFO
#define UART_TX_DATA_STATE_0 UART_TX_DATA_STATE_WITH_FIFO(0)
#else
#define UART_TX_DATA_STATE_0 UART_TX_DATA_STATE_NO_FIFO(0)
#endif  // HAS_KINETISK_UART0_FIFO

void uart0_tx_status_isr() {
  Sender *instance = txInstances[0];

  uint8_t status = UART0_S1;
  uint8_t control = UART0_C2;

  UART_TX(0)

  UART_TX_COMPLETE(0)
}

// ---------------------------------------------------------------------------
//  UART1 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART1_FIFO
#define UART_TX_DATA_STATE_1 UART_TX_DATA_STATE_WITH_FIFO(1)
#else
#define UART_TX_DATA_STATE_1 UART_TX_DATA_STATE_NO_FIFO(1)
#endif  // HAS_KINETISK_UART1_FIFO

void uart1_tx_status_isr() {
  Sender *instance = txInstances[1];

  uint8_t status = UART1_S1;
  uint8_t control = UART1_C2;

  UART_TX(1)

  UART_TX_COMPLETE(1)
}

// ---------------------------------------------------------------------------
//  UART2 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART2_FIFO
#define UART_TX_DATA_STATE_2 UART_TX_DATA_STATE_WITH_FIFO(2)
#else
#define UART_TX_DATA_STATE_2 UART_TX_DATA_STATE_NO_FIFO(2)
#endif  // HAS_KINETISK_UART2_FIFO

void uart2_tx_status_isr() {
  Sender *instance = txInstances[2];

  uint8_t status = UART2_S1;
  uint8_t control = UART2_C2;

  UART_TX(2)

  UART_TX_COMPLETE(2)
}

// ---------------------------------------------------------------------------
//  UART3 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART3

#define UART_TX_DATA_STATE_3 UART_TX_DATA_STATE_NO_FIFO(3)

void uart3_tx_status_isr() {
  Sender *instance = txInstances[3];

  uint8_t status = UART3_S1;
  uint8_t control = UART3_C2;

  UART_TX(3)

  UART_TX_COMPLETE(3)
}
#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART4

#define UART_TX_DATA_STATE_4 UART_TX_DATA_STATE_NO_FIFO(4)

void uart4_tx_status_isr() {
  Sender *instance = txInstances[4];

  uint8_t status = UART4_S1;
  uint8_t control = UART4_C2;

  UART_TX(4)

  UART_TX_COMPLETE(4)
}
#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART5

#define UART_TX_DATA_STATE_5 UART_TX_DATA_STATE_NO_FIFO(5)

void uart5_tx_status_isr() {
  Sender *instance = txInstances[5];

  uint8_t status = UART5_S1;
  uint8_t control = UART5_C2;

  UART_TX(5)

  UART_TX_COMPLETE(5)
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
        // Pause management
        if (instance->paused_) {
          LPUART0_CTRL = LPUART_CTRL_TX_INACTIVE;
          return;
        }
        if (instance->resumeCounter_ > 0) {
          if (--instance->resumeCounter_ == 0) {
            instance->paused_ = true;
          }
        }

        instance->transmitting_ = true;
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = instance->timeSinceBreak_;
        if (timeSinceBreak < instance->breakToBreakTime_) {
          LPUART0_CTRL = LPUART_CTRL_TX_INACTIVE;
          if (instance->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!instance->refreshRateTimer_.begin(
                    []() {
                      txInstances[5]->refreshRateTimer_.end();
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
