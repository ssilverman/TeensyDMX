#include "TeensyDMX.h"

// C++ includes
#include <cmath>
#include <cstring>

// Project includes
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
// start code) at full speed.
//
// Some other timing options:
// 8N2: 1000000/11 (90909) baud, 99us break, 22us MAB
// 8E2: 100000 baud, 100us break, 20us MAB
// 8N1: 50000 baud, 180us break, 20us MAB <-- Closer to "typical" in ANSI E1.11
// 8E1: 45500 baud, 220us break, 22us MAB

constexpr uint32_t kBreakBaud   = 50000;       // 20us
constexpr uint32_t kBreakFormat = SERIAL_8N1;  // 9:1
constexpr uint32_t kSlotsBaud   = 250000;      // 4us
constexpr uint32_t kSlotsFormat = SERIAL_8N2;  // 9:2

// TX control states
#define UART_C2_TX_ENABLE         (UART_C2_TE)
#define UART_C2_TX_ACTIVE         ((UART_C2_TX_ENABLE) | (UART_C2_TIE))
#define UART_C2_TX_COMPLETING     ((UART_C2_TX_ENABLE) | (UART_C2_TCIE))
#define UART_C2_TX_INACTIVE       (UART_C2_TX_ENABLE)
#define LPUART_CTRL_TX_ENABLE     (LPUART_CTRL_TE)
#define LPUART_CTRL_TX_ACTIVE     ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TIE))
#define LPUART_CTRL_TX_COMPLETING ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TCIE))
#define LPUART_CTRL_TX_INACTIVE   (LPUART_CTRL_TX_ENABLE)

// Used by the TX ISRs
Sender *volatile txInstances[6]{nullptr};

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
      transmitting_(false),
      doneTXFunc_{nullptr} {}

Sender::~Sender() {
  end();
}

#define ACTIVATE_TX_SERIAL(N)\
  attachInterruptVector(IRQ_UART##N##_STATUS, uart##N##_tx_isr);\
  UART##N##_C2 = UART_C2_TX_ACTIVE;

void Sender::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  if (serialIndex_ < 0) {
    return;
  }

  // Set up the instance for the ISRs
  Sender *s = txInstances[serialIndex_];
  txInstances[serialIndex_] = this;
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

  // Remove any chance that our TX ISRs start after end() is called,
  // so disable the IRQs first

  uart_.end();
  refreshRateTimer_.end();

  // Remove the reference from the instances,
  // but only if we're the ones who added it
  if (txInstances[serialIndex_] == this) {
    txInstances[serialIndex_] = nullptr;
  }
}

// memcpy implementation that accepts a volatile destination.
// Derived from:
// https://github.com/ARM-software/arm-trusted-firmware/blob/master/lib/libc/memcpy.c
volatile void *memcpy(volatile void *dst, const void *src, size_t len) {
  volatile uint8_t *d = reinterpret_cast<volatile uint8_t *>(dst);
  const uint8_t *s = reinterpret_cast<const uint8_t *>(src);

  while (len-- != 0) {
    *(d++) = *(s++);
  }

  return dst;
}

void Sender::set(int channel, uint8_t value) {
  if (channel < 0 || kMaxDMXPacketSize <= channel) {
    return;
  }
  if (!paused_) {
    outputBuf_[channel] = value;
  }
  pausedBuf_[channel] = value;
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

  if (!paused_) {
    memcpy(&outputBuf_[startChannel], values, len);
  }
  memcpy(&pausedBuf_[startChannel], values, len);
}

void Sender::setRefreshRate(float rate) {
  if ((rate != rate) || rate < 0.0f) {  // NaN or negative
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
  resumeFor(n, doneTXFunc_);
}

void Sender::resumeFor(int n, void (*doneTXFunc)(Sender *s)) {
  if (n < 0) {
    return;
  }

  // Pausing made transmission INACTIVE
  disableIRQs();
  //{
    resumeCounter_ = n;
    if (paused_) {
      if (!transmitting_) {
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

      // Copy whatever's in the paused buffer to the output buffer
      memcpy(outputBuf_, pausedBuf_, kMaxDMXPacketSize);

      paused_ = false;
    }
    doneTXFunc_ = doneTXFunc;
  //}
  enableIRQs();
}

bool Sender::isTransmitting() const {
  // Check these both atomically
  disableIRQs();
  //{
    bool state = !paused_ || transmitting_;
  //}
  enableIRQs();
  return state;
}

void Sender::completePacket() {
  packetCount_++;
  outputBufIndex_ = 0;
  transmitting_ = false;
  state_ = XmitStates::kIdle;

  if (paused_ && doneTXFunc_ != nullptr) {
    doneTXFunc_(this);
  }
}

// ---------------------------------------------------------------------------
//  IRQ management
// ---------------------------------------------------------------------------

void Sender::disableIRQs() const {
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
}

void Sender::enableIRQs() const {
  switch (serialIndex_) {
    case 0:
      NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
      break;
    case 1:
      NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);
      break;
    case 2:
      NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);
      break;
#ifdef HAS_KINETISK_UART3
    case 3:
      NVIC_ENABLE_IRQ(IRQ_UART3_STATUS);
      break;
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
    case 4:
      NVIC_ENABLE_IRQ(IRQ_UART4_STATUS);
      break;
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
    case 5:
      NVIC_ENABLE_IRQ(IRQ_UART5_STATUS);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      NVIC_ENABLE_IRQ(IRQ_LPUART0);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }
}

// ---------------------------------------------------------------------------
//  UART0 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART0_FIFO
#define UART_TX_DATA_STATE_0 UART_TX_DATA_STATE_WITH_FIFO(0)
#else
#define UART_TX_DATA_STATE_0 \
  UART_TX_DATA_STATE_NO_FIFO(UART0_C2, UART0_D, UART_C2)
#endif  // HAS_KINETISK_UART0_FIFO

void uart0_tx_isr() {
  uint8_t status = UART0_S1;
  uint8_t control = UART0_C2;

  UART_TX(0, UART0_C2, UART0_D, UART_C2, UART_S1)

  UART_TX_COMPLETE(UART0_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_0

// ---------------------------------------------------------------------------
//  UART1 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART1_FIFO
#define UART_TX_DATA_STATE_1 UART_TX_DATA_STATE_WITH_FIFO(1)
#else
#define UART_TX_DATA_STATE_1 \
  UART_TX_DATA_STATE_NO_FIFO(UART1_C2, UART1_D, UART_C2)
#endif  // HAS_KINETISK_UART1_FIFO

void uart1_tx_isr() {
  uint8_t status = UART1_S1;
  uint8_t control = UART1_C2;

  UART_TX(1, UART1_C2, UART1_D, UART_C2, UART_S1)

  UART_TX_COMPLETE(UART1_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_1

// ---------------------------------------------------------------------------
//  UART2 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART2_FIFO
#define UART_TX_DATA_STATE_2 UART_TX_DATA_STATE_WITH_FIFO(2)
#else
#define UART_TX_DATA_STATE_2 \
  UART_TX_DATA_STATE_NO_FIFO(UART2_C2, UART2_D, UART_C2)
#endif  // HAS_KINETISK_UART2_FIFO

void uart2_tx_isr() {
  uint8_t status = UART2_S1;
  uint8_t control = UART2_C2;

  UART_TX(2, UART2_C2, UART2_D, UART_C2, UART_S1)

  UART_TX_COMPLETE(UART2_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_2

// ---------------------------------------------------------------------------
//  UART3 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART3

#define UART_TX_DATA_STATE_3 \
  UART_TX_DATA_STATE_NO_FIFO(UART3_C2, UART3_D, UART_C2)

void uart3_tx_isr() {
  uint8_t status = UART3_S1;
  uint8_t control = UART3_C2;

  UART_TX(3, UART3_C2, UART3_D, UART_C2, UART_S1)

  UART_TX_COMPLETE(UART3_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_3

#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART4

#define UART_TX_DATA_STATE_4 \
  UART_TX_DATA_STATE_NO_FIFO(UART4_C2, UART4_D, UART_C2)

void uart4_tx_isr() {
  uint8_t status = UART4_S1;
  uint8_t control = UART4_C2;

  UART_TX(4, UART4_C2, UART4_D, UART_C2, UART_S1)

  UART_TX_COMPLETE(UART4_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_4

#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 TX ISR
// ---------------------------------------------------------------------------

#ifdef HAS_KINETISK_UART5

#define UART_TX_DATA_STATE_5 \
  UART_TX_DATA_STATE_NO_FIFO(UART5_C2, UART5_D, UART_C2)

void uart5_tx_isr() {
  uint8_t status = UART5_S1;
  uint8_t control = UART5_C2;

  UART_TX(5, UART5_C2, UART5_D, UART_C2, UART_S1)

  UART_TX_COMPLETE(UART5_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_5

#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 TX ISR
// ---------------------------------------------------------------------------

#define UART_TX_DATA_STATE_5 \
  UART_TX_DATA_STATE_NO_FIFO(LPUART0_CTRL, LPUART0_DATA, LPUART_CTRL)

#ifdef HAS_KINETISK_LPUART0
void lpuart0_tx_isr() {
  uint32_t status = LPUART0_STAT;
  uint32_t control = LPUART0_CTRL;

  UART_TX(5, LPUART0_CTRL, LPUART0_DATA, LPUART_CTRL, LPUART_STAT)

  UART_TX_COMPLETE(LPUART0_CTRL, LPUART_CTRL, LPUART_STAT)
}
#endif  // HAS_KINETISK_LPUART0

}  // namespace teensydmx
}  // namespace qindesign
