// This file is part of the TeensyDMX library.
// (c) 2017-2019 Shawn Silverman

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
// the minimum BREAK and Mark after BREAK (MAB) times are
// 92us and 12us, respectively.
//
// If we assume 12us is the length of a stop bit, then 1/12us ≈ 83333 baud.
// For 8N1, the length of the 9 bits before the stop bit ≈ 108us.
//
// Minimum accepted receive BREAK-to-BREAK time = 1196us.
// This means that we must transmit at least 24 slots (25 including the
// start code) at full speed.
//
// Some other timing options:
// 8N2: 1000000/11 (90909) baud, 99us BREAK, 22us MAB
// 8E2: 100000 baud, 100us BREAK, 20us MAB
// 8N1: 50000 baud, 180us BREAK, 20us MAB <-- Closer to "typical" in ANSI E1.11
// 8E1: 45500 baud, 220us BREAK, 22us MAB

constexpr uint32_t kBreakBaud   = 50000;       // 20us
constexpr uint32_t kBreakFormat = SERIAL_8N1;  // 9:1
constexpr uint32_t kSlotsBaud   = 250000;      // 4us
constexpr uint32_t kSlotsFormat = SERIAL_8N2;  // 9:2

constexpr uint32_t kSerialBreakTime = 1000000/kBreakBaud * 9;  // In us
constexpr uint32_t kSerialMABTime   = 1000000/kBreakBaud * 1;  // In us

// Empirically observed MAB generation adjustment constants, for 20us
// The timer adjust values are subtracted from the requested MAB to get the
// actual MAB. The "delay adjusted adjust" is a value that's added to the
// adjusted delay when requesting delayMicroseconds if the second timer call to
// create the MAB fails.
#if defined(__MK20DX128__) || defined(__MK20DX256__)
constexpr uint32_t kMABBaudAdjust = 9;
constexpr uint32_t kMABTimerAdjust = 5;  // 5.5, 5-8 works
constexpr uint32_t kMABDelayAdjustedAdjust = 0;
#elif defined(__MKL26Z64__)
constexpr uint32_t kMABBaudAdjust = 6;
constexpr uint32_t kMABTimerAdjust = 9;  // Ranges from 9-12, (5)8(9 best)-10
constexpr uint32_t kMABDelayAdjustedAdjust = 2;
#elif defined(__MK64FX512__)
constexpr uint32_t kMABBaudAdjust = 5;
constexpr uint32_t kMABTimerAdjust = 2;  // Ranges from 7.25-7.5, 2-4 works
constexpr uint32_t kMABDelayAdjustedAdjust = 1;
#elif defined(__MK66FX1M0__)
constexpr uint32_t kMABBaudAdjust = 5;
constexpr uint32_t kMABTimerAdjust = 2;  // 5.75, 2-5 works
constexpr uint32_t kMABDelayAdjustedAdjust = 1;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
constexpr uint32_t kMABBaudAdjust = 4;   // 4.5
constexpr uint32_t kMABTimerAdjust = 0;  // 2.25
constexpr uint32_t kMABDelayAdjustedAdjust = 0;
#else
constexpr uint32_t kMABBaudAdjust = 0;
constexpr uint32_t kMABTimerAdjust = 0;
constexpr uint32_t kMABDelayAdjustedAdjust = 0;
#endif

// Used by the TX ISRs
Sender *volatile txInstances[6]{nullptr};

Sender::Sender(HardwareSerial &uart)
    : TeensyDMX(uart),
      lpuartBreakParams_{},
      lpuartSlotsParams_{},
      lpuartParamsSet_(false),
      began_(false),
      state_(XmitStates::kIdle),
      outputBuf_{0},
      outputBufIndex_(0),
      breakTime_(kSerialBreakTime),
      mabTime_(kSerialMABTime),
      adjustedMABTime_(mabTime_),
      packetSize_(kMaxDMXPacketSize),
      refreshRate_(INFINITY),
      breakToBreakTime_(0),
      timeSinceBreak_{0},
      paused_(false),
      resumeCounter_(0),
      transmitting_(false),
      doneTXFunc_{nullptr} {
  setMABTime(kSerialMABTime);
}

Sender::~Sender() {
  end();
}

// TX control states
#define UART_C2_TX_ENABLE         (UART_C2_TE)
#define UART_C2_TX_ACTIVE         ((UART_C2_TX_ENABLE) | (UART_C2_TIE))
#define UART_C2_TX_COMPLETING     ((UART_C2_TX_ENABLE) | (UART_C2_TCIE))
#define UART_C2_TX_INACTIVE       (UART_C2_TX_ENABLE)
#define LPUART_CTRL_TX_ENABLE     (LPUART_CTRL_TE)
#define LPUART_CTRL_TX_ACTIVE     ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TIE))
#define LPUART_CTRL_TX_COMPLETING ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TCIE))
#define LPUART_CTRL_TX_INACTIVE   (LPUART_CTRL_TX_ENABLE)

#define ACTIVATE_UART_TX_SERIAL(N)                               \
  attachInterruptVector(IRQ_UART##N##_STATUS, uart##N##_tx_isr); \
  UART##N##_C2 = UART_C2_TX_ACTIVE;

#define ACTIVATE_LPUART_TX_SERIAL(N)                        \
  attachInterruptVector(IRQ_LPUART##N, lpuart##N##_tx_isr); \
  LPUART##N##_CTRL = LPUART_CTRL_TX_ACTIVE;

// Gleans the LPUART parameters. The BREAK baud rate and format is expected to
// have been set.
#define GLEAN_LPUART_PARAMS(N)                                \
  if (!lpuartParamsSet_) {                                    \
    lpuartSlotsParams_ = {LPUART##N##_BAUD, LPUART##N##_STAT, \
                          LPUART##N##_CTRL};                  \
    uart_.begin(kBreakBaud, kBreakFormat);                    \
    lpuartBreakParams_ = {LPUART##N##_BAUD, LPUART##N##_STAT, \
                          LPUART##N##_CTRL};                  \
    /* Put it back so that the code is consistent */          \
    uart_.begin(kSlotsBaud, kSlotsFormat);                    \
    lpuartParamsSet_ = true;                                  \
  }

void Sender::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  if (serialIndex_ < 0) {
    return;
  }

  // Reset all the stats
  resetPacketCount();

  // Set up the instance for the ISRs
  Sender *s = txInstances[serialIndex_];
  txInstances[serialIndex_] = this;
  if (s != nullptr && s != this) {  // NOTE: Shouldn't be able to be 'this'
    s->end();
  }

  transmitting_ = false;
  state_ = XmitStates::kIdle;
  uart_.begin(kSlotsBaud, kSlotsFormat);

  // Also set the interval timer priority to match the UART priority

  switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
    case 0:
      ACTIVATE_UART_TX_SERIAL(0)
      intervalTimer_.priority(NVIC_GET_PRIORITY(IRQ_UART0_STATUS));
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
    case 1:
      ACTIVATE_UART_TX_SERIAL(1)
      intervalTimer_.priority(NVIC_GET_PRIORITY(IRQ_UART1_STATUS));
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
    case 2:
      ACTIVATE_UART_TX_SERIAL(2)
      intervalTimer_.priority(NVIC_GET_PRIORITY(IRQ_UART2_STATUS));
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      ACTIVATE_UART_TX_SERIAL(3)
      intervalTimer_.priority(NVIC_GET_PRIORITY(IRQ_UART3_STATUS));
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
    case 4:
      ACTIVATE_UART_TX_SERIAL(4)
      intervalTimer_.priority(NVIC_GET_PRIORITY(IRQ_UART4_STATUS));
      break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
    case 5:
      ACTIVATE_UART_TX_SERIAL(5)
      intervalTimer_.priority(NVIC_GET_PRIORITY(IRQ_UART5_STATUS));
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      GLEAN_LPUART_PARAMS(0)
      ACTIVATE_LPUART_TX_SERIAL(0)
      intervalTimer_.priority(NVIC_GET_PRIORITY(IRQ_LPUART0));
      break;
#endif  // HAS_KINETISK_LPUART0 || HAS_KINETISK_UART5
  }
}

// Undefine these macros
#undef ACTIVATE_UART_TX_SERIAL
#undef ACTIVATE_LPUART_TX_SERIAL

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
  intervalTimer_.end();

  // Remove the reference from the instances,
  // but only if we're the ones who added it
  if (txInstances[serialIndex_] == this) {
    txInstances[serialIndex_] = nullptr;
  }
}

void Sender::setBreakTime(uint32_t t) {
  breakTime_ = t;
}

void Sender::setMABTime(uint32_t t) {
  mabTime_ = t;
  if (t < kMABTimerAdjust) {
    adjustedMABTime_ = 0;
  } else {
    adjustedMABTime_ = t - kMABTimerAdjust;
  }
}

// memcpy implementation that accepts a volatile destination.
// Derived from:
// https://github.com/ARM-software/arm-trusted-firmware/blob/master/lib/libc/memcpy.c
volatile void *memcpy(volatile void *dst, const void *src, size_t len) {
  volatile char *d = reinterpret_cast<volatile char *>(dst);
  const char *s = reinterpret_cast<const char *>(src);

  while (len-- != 0) {
    *(d++) = *(s++);
  }

  return dst;
}

void Sender::set(int channel, uint8_t value) {
  if (channel < 0 || kMaxDMXPacketSize <= channel) {
    return;
  }
  outputBuf_[channel] = value;
}

void Sender::set16Bit(int channel, uint16_t value) {
  if (channel < 0 || kMaxDMXPacketSize - 1 <= channel) {
    return;
  }

  Lock lock{*this};
  //{
    outputBuf_[channel] = value >> 8;
    outputBuf_[channel + 1] = value;
  //}
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

  Lock lock{*this};
  //{
    memcpy(&outputBuf_[startChannel], values, len);
  //}
}

void Sender::clear() {
  Lock lock{*this};
  //{
    volatile uint8_t *b = outputBuf_;
    for (int i = 0; i < kMaxDMXPacketSize; i++) {
      *(b++) = 0;
    }
  //}
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
  Lock lock{*this};
  //{
    resumeCounter_ = n;
    if (paused_) {
      if (!transmitting_) {
        switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
          case 0:
            UART0_C2 = UART_C2_TX_ACTIVE;
            break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
          case 1:
            UART1_C2 = UART_C2_TX_ACTIVE;
            break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
          case 2:
            UART2_C2 = UART_C2_TX_ACTIVE;
            break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
          case 3:
            UART3_C2 = UART_C2_TX_ACTIVE;
            break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
          case 4:
            UART4_C2 = UART_C2_TX_ACTIVE;
            break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
          case 5:
            UART5_C2 = UART_C2_TX_ACTIVE;
            break;
#elif defined(HAS_KINETISK_LPUART0)
          case 5:
            LPUART0_CTRL = LPUART_CTRL_TX_ACTIVE;
            break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
        }
      }

      paused_ = false;
    }
    doneTXFunc_ = doneTXFunc;
  //}
}

bool Sender::isTransmitting() const {
  // Check these both atomically
  Lock lock{*this};
  //{
    bool state = !paused_ || transmitting_;
  //}
  return state;
}

void Sender::completePacket() {
  incPacketCount();
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
  if (!began_) {
    return;
  }
  switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
    case 0:
      NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
    case 1:
      NVIC_DISABLE_IRQ(IRQ_UART1_STATUS);
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
    case 2:
      NVIC_DISABLE_IRQ(IRQ_UART2_STATUS);
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      NVIC_DISABLE_IRQ(IRQ_UART3_STATUS);
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
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
  if (!began_) {
    return;
  }
  switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
    case 0:
      NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
    case 1:
      NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
    case 2:
      NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      NVIC_ENABLE_IRQ(IRQ_UART3_STATUS);
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
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

#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)

#if defined(HAS_KINETISK_UART0_FIFO)
#define UART_TX_DATA_STATE_0 UART_TX_DATA_STATE_WITH_FIFO(0)
#else
#define UART_TX_DATA_STATE_0 \
  UART_TX_DATA_STATE_NO_FIFO(UART0_C2, UART0_D, UART_C2)
#endif  // HAS_KINETISK_UART0_FIFO

#if defined(KINETISK)
#define UART_TX_SET_BREAK_BAUD_0 \
  KINETISK_SET_BAUD(0, BAUD2DIV(kBreakBaud), serial_format(kBreakFormat))
#define UART_TX_SET_SLOTS_BAUD_0 \
  KINETISK_SET_BAUD(0, BAUD2DIV(kSlotsBaud), serial_format(kSlotsFormat))
#else
#define UART_TX_SET_BREAK_BAUD_0 \
  KINETISL_SET_BAUD(0, BAUD2DIV(kBreakBaud), (kBreakFormat & SERIAL_2STOP_BITS) != 0)
#define UART_TX_SET_SLOTS_BAUD_0 \
  KINETISL_SET_BAUD(0, BAUD2DIV(kSlotsBaud), true)
#endif  // KINETISK || other

void uart0_tx_isr() {
  uint8_t status = UART0_S1;
  uint8_t control = UART0_C2;

  UART_TX(0, 0, UART0_C2, UART0_C3, UART0_D, UART_C2, UART_C3, UART_S1)

  UART_TX_COMPLETE(0, UART0_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_0
#undef UART_TX_SET_BREAK_BAUD_0
#undef UART_TX_SET_SLOTS_BAUD_0

#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

// ---------------------------------------------------------------------------
//  UART1 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)

#if defined(HAS_KINETISK_UART1_FIFO)
#define UART_TX_DATA_STATE_1 UART_TX_DATA_STATE_WITH_FIFO(1)
#else
#define UART_TX_DATA_STATE_1 \
  UART_TX_DATA_STATE_NO_FIFO(UART1_C2, UART1_D, UART_C2)
#endif  // HAS_KINETISK_UART1_FIFO

#if defined(KINETISK)
#define UART_TX_SET_BREAK_BAUD_1 \
  KINETISK_SET_BAUD(1, BAUD2DIV2(kBreakBaud), serial2_format(kBreakFormat))
#define UART_TX_SET_SLOTS_BAUD_1 \
  KINETISK_SET_BAUD(1, BAUD2DIV2(kSlotsBaud), serial2_format(kSlotsFormat))
#else
#define UART_TX_SET_BREAK_BAUD_1 \
  KINETISL_SET_BAUD(1, BAUD2DIV2(kBreakBaud), (kBreakFormat & SERIAL_2STOP_BITS) != 0)
#define UART_TX_SET_SLOTS_BAUD_1 \
  KINETISL_SET_BAUD(1, BAUD2DIV2(kSlotsBaud), true)
#endif  // KINETISK || other

void uart1_tx_isr() {
  uint8_t status = UART1_S1;
  uint8_t control = UART1_C2;

  UART_TX(1, 1, UART1_C2, UART1_C3, UART1_D, UART_C2, UART_C3, UART_S1)

  UART_TX_COMPLETE(1, UART1_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_1
#undef UART_TX_SET_BREAK_BAUD_1
#undef UART_TX_SET_SLOTS_BAUD_1

#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

// ---------------------------------------------------------------------------
//  UART2 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)

#define UART_TX_DATA_STATE_2 \
  UART_TX_DATA_STATE_NO_FIFO(UART2_C2, UART2_D, UART_C2)

#if defined(KINETISK)
#define UART_TX_SET_BREAK_BAUD_2 \
  KINETISK_SET_BAUD(2, BAUD2DIV3(kBreakBaud), serial3_format(kBreakFormat))
#define UART_TX_SET_SLOTS_BAUD_2 \
  KINETISK_SET_BAUD(2, BAUD2DIV3(kSlotsBaud), serial3_format(kSlotsFormat))
#else
#define UART_TX_SET_BREAK_BAUD_2 \
  KINETISL_SET_BAUD(2, BAUD2DIV3(kBreakBaud), (kBreakFormat & SERIAL_2STOP_BITS) != 0)
#define UART_TX_SET_SLOTS_BAUD_2 \
  KINETISL_SET_BAUD(2, BAUD2DIV3(kSlotsBaud), true)
#endif  // KINETISK || other

void uart2_tx_isr() {
  uint8_t status = UART2_S1;
  uint8_t control = UART2_C2;

  UART_TX(2, 2, UART2_C2, UART2_C3, UART2_D, UART_C2, UART_C3, UART_S1)

  UART_TX_COMPLETE(2, UART2_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_2
#undef UART_TX_SET_BREAK_BAUD_2
#undef UART_TX_SET_SLOTS_BAUD_2

#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

// ---------------------------------------------------------------------------
//  UART3 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART3)

#define UART_TX_DATA_STATE_3 \
  UART_TX_DATA_STATE_NO_FIFO(UART3_C2, UART3_D, UART_C2)

#define UART_TX_SET_BREAK_BAUD_3 \
  KINETISK_SET_BAUD(3, BAUD2DIV3(kBreakBaud), serial4_format(kBreakFormat))
#define UART_TX_SET_SLOTS_BAUD_3 \
  KINETISK_SET_BAUD(3, BAUD2DIV3(kSlotsBaud), serial4_format(kSlotsFormat))

void uart3_tx_isr() {
  uint8_t status = UART3_S1;
  uint8_t control = UART3_C2;

  UART_TX(3, 3, UART3_C2, UART3_C3, UART3_D, UART_C2, UART_C3, UART_S1)

  UART_TX_COMPLETE(3, UART3_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_3
#undef UART_TX_SET_BREAK_BAUD_3
#undef UART_TX_SET_SLOTS_BAUD_3

#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART4)

#define UART_TX_DATA_STATE_4 \
  UART_TX_DATA_STATE_NO_FIFO(UART4_C2, UART4_D, UART_C2)

#define UART_TX_SET_BREAK_BAUD_4 \
  KINETISK_SET_BAUD(4, BAUD2DIV3(kBreakBaud), serial5_format(kBreakFormat))
#define UART_TX_SET_SLOTS_BAUD_4 \
  KINETISK_SET_BAUD(4, BAUD2DIV3(kSlotsBaud), serial5_format(kSlotsFormat))

void uart4_tx_isr() {
  uint8_t status = UART4_S1;
  uint8_t control = UART4_C2;

  UART_TX(4, 4, UART4_C2, UART4_C3, UART4_D, UART_C2, UART_C3, UART_S1)

  UART_TX_COMPLETE(4, UART4_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_4
#undef UART_TX_SET_BREAK_BAUD_4
#undef UART_TX_SET_SLOTS_BAUD_4

#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART5)

#define UART_TX_DATA_STATE_5 \
  UART_TX_DATA_STATE_NO_FIFO(UART5_C2, UART5_D, UART_C2)

#define UART_TX_SET_BREAK_BAUD_5 \
  KINETISK_SET_BAUD(5, BAUD2DIV3(kBreakBaud), serial6_format(kBreakFormat))
#define UART_TX_SET_SLOTS_BAUD_5 \
  KINETISK_SET_BAUD(5, BAUD2DIV3(kSlotsBaud), serial6_format(kSlotsFormat))

void uart5_tx_isr() {
  uint8_t status = UART5_S1;
  uint8_t control = UART5_C2;

  UART_TX(5, 5, UART5_C2, UART5_C3, UART5_D, UART_C2, UART_C3, UART_S1)

  UART_TX_COMPLETE(5, UART5_C2, UART_C2, UART_S1)
}

#undef UART_TX_DATA_STATE_5
#undef UART_TX_SET_BREAK_BAUD_5
#undef UART_TX_SET_SLOTS_BAUD_5

#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_LPUART0)

#define UART_TX_DATA_STATE_0 \
  UART_TX_DATA_STATE_NO_FIFO(LPUART0_CTRL, LPUART0_DATA, LPUART_CTRL)

#define UART_TX_SET_BREAK_BAUD_0 LPUART_SET_BAUD(0, lpuartBreakParams_)
#define UART_TX_SET_SLOTS_BAUD_0 LPUART_SET_BAUD(0, lpuartSlotsParams_)

void lpuart0_tx_isr() {
  uint32_t status = LPUART0_STAT;
  uint32_t control = LPUART0_CTRL;

  UART_TX(5, 0, LPUART0_CTRL, LPUART0_CTRL, LPUART0_DATA, LPUART_CTRL,
          LPUART_CTRL, LPUART_STAT)

  UART_TX_COMPLETE(0, LPUART0_CTRL, LPUART_CTRL, LPUART_STAT)
}

#undef UART_TX_DATA_STATE_0
#undef UART_TX_SET_BREAK_BAUD_0
#undef UART_TX_SET_SLOTS_BAUD_0

#endif  // HAS_KINETISK_LPUART0

// Undefine these macros
#undef UART_C2_TX_ENABLE
#undef UART_C2_TX_ACTIVE
#undef UART_C2_TX_COMPLETING
#undef UART_C2_TX_INACTIVE
#undef LPUART_CTRL_TX_ENABLE
#undef LPUART_CTRL_TX_ACTIVE
#undef LPUART_CTRL_TX_COMPLETING
#undef LPUART_CTRL_TX_INACTIVE

}  // namespace teensydmx
}  // namespace qindesign
