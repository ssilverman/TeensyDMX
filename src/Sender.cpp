// This file is part of the TeensyDMX library.
// (c) 2017-2021 Shawn Silverman

#include "TeensyDMX.h"

// C++ includes
#include <algorithm>
#include <limits>

namespace qindesign {
namespace teensydmx {

// Notes on transmit timing:
// According to https://en.wikipedia.org/wiki/DMX512,
// the minimum BREAK and MARK after BREAK (MAB) times are
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

constexpr uint32_t kDefaultBreakBaud   = 50000;       // 20us
constexpr uint32_t kDefaultBreakFormat = SERIAL_8N1;  // 9:1

constexpr uint32_t kDefaultBreakTime = 180;  // In us
constexpr uint32_t kDefaultMABTime   = 20;   // In us

// Modifier bits in the serial format
constexpr uint32_t kSerialFormatRXINVBit = 0x10;
constexpr uint32_t kSerialFormatTXINVBit = 0x20;

#ifndef TEENSYDMX_USE_PERIODICTIMER
// Empirically observed BREAK generation adjustment constants, for 180us. The
// timer adjust values are added to the requested BREAK to get the actual BREAK.
#if defined(__MK20DX128__) || defined(__MK20DX256__)
constexpr uint32_t kBreakTimerAdjust = 1;
#elif defined(__MKL26Z64__)
constexpr uint32_t kBreakTimerAdjust = 5;
#elif defined(__MK64FX512__)
constexpr uint32_t kBreakTimerAdjust = 1;
#elif defined(__MK66FX1M0__)
constexpr uint32_t kBreakTimerAdjust = 1;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
constexpr uint32_t kBreakTimerAdjust = 0;
#else
constexpr uint32_t kBreakTimerAdjust = 0;
#endif  // Which chip?
#endif  // !TEENSYDMX_USE_PERIODICTIMER

// Empirically observed MAB generation adjustment constants, for 20us. The timer
// adjust values are subtracted from the requested MAB to get the actual MAB.
#if defined(__MK20DX128__) || defined(__MK20DX256__)
constexpr uint32_t kMABTimerAdjust = 7;
#elif defined(__MKL26Z64__)
constexpr uint32_t kMABTimerAdjust = 12;
#elif defined(__MK64FX512__)
constexpr uint32_t kMABTimerAdjust = 5;
#elif defined(__MK66FX1M0__)
constexpr uint32_t kMABTimerAdjust = 4;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
constexpr uint32_t kMABTimerAdjust = 1;
#else
constexpr uint32_t kMABTimerAdjust = 0;
#endif  // Which chip?

// Empirically observed inter-slot timer adjustment constants, for 40us. The
// timer adjust values are subtracted from the requested value to get the
// actual value.
#if defined(__MK20DX128__) || defined(__MK20DX256__)
constexpr uint32_t kInterSlotTimerAdjust = 9;
#elif defined(__MKL26Z64__)
constexpr uint32_t kInterSlotTimerAdjust = 23;
#elif defined(__MK64FX512__)
constexpr uint32_t kInterSlotTimerAdjust = 9;
#elif defined(__MK66FX1M0__)
constexpr uint32_t kInterSlotTimerAdjust = 6;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
constexpr uint32_t kInterSlotTimerAdjust = 2;
#else
constexpr uint32_t kInterSlotTimerAdjust = 0;
#endif  // Which chip?

// Empirically observed MBB timer adjustment constants. The timer adjust values
// are subtracted from the requested value to get the actual value.
// kMBBTimerMin: When the delay is zero
// kMBBTimerMinAdjusted: Minimum delay for there not to be an error
//     Some delays don't cause a delay for some reason. Perhaps this is because
//     the timer is reused too quickly? Or maybe it's due to the design
//     of PeriodicTimer?
#if defined(__MK20DX128__) || defined(__MK20DX256__)
constexpr uint32_t kMBBTimerMin = 24;
constexpr uint32_t kMBBTimerMinAdjusted = 1;
constexpr uint32_t kMBBTimerAdjust = 29;
#elif defined(__MKL26Z64__)
constexpr uint32_t kMBBTimerMin = 119;
constexpr uint32_t kMBBTimerMinAdjusted = 4;
constexpr uint32_t kMBBTimerAdjust = 133;  // At 4us
#elif defined(__MK64FX512__)
constexpr uint32_t kMBBTimerMin = 19;
constexpr uint32_t kMBBTimerMinAdjusted = 2;
constexpr uint32_t kMBBTimerAdjust = 22;
#elif defined(__MK66FX1M0__)
constexpr uint32_t kMBBTimerMin = 11;
constexpr uint32_t kMBBTimerMinAdjusted = 1;
constexpr uint32_t kMBBTimerAdjust = 13;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
constexpr uint32_t kMBBTimerMin = 2;
constexpr uint32_t kMBBTimerMinAdjusted = 1;
constexpr uint32_t kMBBTimerAdjust = 4;
#else
constexpr uint32_t kMBBTimerMin = 0;
constexpr uint32_t kMBBTimerMinAdjusted = 0;
constexpr uint32_t kMBBTimerAdjust = 0;
#endif  // Which chip?

// TX ISR routines
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
void uart0_tx_isr();
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
void uart1_tx_isr();
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
void uart2_tx_isr();
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
void uart3_tx_isr();
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
void uart4_tx_isr();
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
void uart5_tx_isr();
#endif  // HAS_KINETISK_UART5

#if defined(HAS_KINETISK_LPUART0)
void lpuart0_tx_isr();
#endif  // HAS_KINETISK_LPUART0

#ifdef IMXRT_LPUART6
void lpuart6_tx_isr();
#endif  // IMXRT_LPUART6

#ifdef IMXRT_LPUART4
void lpuart4_tx_isr();
#endif  // IMXRT_LPUART4

#ifdef IMXRT_LPUART2
void lpuart2_tx_isr();
#endif  // IMXRT_LPUART2

#ifdef IMXRT_LPUART3
void lpuart3_tx_isr();
#endif  // IMXRT_LPUART3

#ifdef IMXRT_LPUART8
void lpuart8_tx_isr();
#endif  // IMXRT_LPUART8

#ifdef IMXRT_LPUART1
void lpuart1_tx_isr();
#endif  // IMXRT_LPUART1

#ifdef IMXRT_LPUART7
void lpuart7_tx_isr();
#endif  // IMXRT_LPUART7

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))
void lpuart5_tx_isr();
#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)

// Used by the TX ISRs
#if defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41)
static Sender *volatile txInstances[8]{nullptr};
#else
static Sender *volatile txInstances[7]{nullptr};
#endif  // __IMXRT1052__ || ARDUINO_TEENSY41

Sender::Sender(HardwareSerial &uart)
    : TeensyDMX(uart),
      began_(false),
      state_(XmitStates::kIdle),
      activeBuf_{0},
      inactiveBuf_{0},
      inactiveBufIndex_(0),
      breakTime_(kDefaultBreakTime),
      mabTime_(kDefaultMABTime),
#ifndef TEENSYDMX_USE_PERIODICTIMER
      adjustedBreakTime_(breakTime_),
#endif  // !TEENSYDMX_USE_PERIODICTIMER
      adjustedMABTime_(mabTime_),
      breakBaud_(kDefaultBreakBaud),
      breakFormat_(kDefaultBreakFormat),
      breakUseTimer_(false),
      interSlotTime_(0),
      adjustedInterSlotTime_(0),
      activePacketSize_(kMaxDMXPacketSize),
      inactivePacketSize_(kMaxDMXPacketSize),
      mbbTime_(0),
      adjustedMBBTime_(0),
      refreshRate_(std::numeric_limits<float>::infinity()),
      breakToBreakTime_(0),
      breakStartTime_(0),
      paused_(false),
      resumeCounter_(0),
      transmitting_(false),
      doneTXFunc_{nullptr} {
#ifndef TEENSYDMX_USE_PERIODICTIMER
  setBreakTime(breakTime_);
#endif  // !TEENSYDMX_USE_PERIODICTIMER
  setMABTime(mabTime_);

  switch(serialIndex_) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
    case 0:
      sendHandler_ = std::make_unique<UARTSendHandler>(
          serialIndex_, this, &KINETISK_UART0, IRQ_UART0_STATUS, &uart0_tx_isr);
      break;
#elif defined(IMXRT_LPUART6)
    case 0:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART6, IRQ_LPUART6, lpuart6_tx_isr);
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0 || IMXRT_LPUART6

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
    case 1:
      sendHandler_ = std::make_unique<UARTSendHandler>(
          serialIndex_, this, &KINETISK_UART1, IRQ_UART1_STATUS, &uart1_tx_isr);
      break;
#elif defined(IMXRT_LPUART4)
    case 1:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART4, IRQ_LPUART4, lpuart4_tx_isr);
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1 || IMXRT_LPUART4

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
    case 2:
      sendHandler_ = std::make_unique<UARTSendHandler>(
          serialIndex_, this, &KINETISK_UART2, IRQ_UART2_STATUS, &uart2_tx_isr);
      break;
#elif defined(IMXRT_LPUART2)
    case 2:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART2, IRQ_LPUART2, lpuart2_tx_isr);
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2 || IMXRT_LPUART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      sendHandler_ = std::make_unique<UARTSendHandler>(
          serialIndex_, this, &KINETISK_UART3, IRQ_UART3_STATUS, &uart3_tx_isr);
      break;
#elif defined(IMXRT_LPUART3)
    case 3:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART3, IRQ_LPUART3, lpuart3_tx_isr);
      break;
#endif  // HAS_KINETISK_UART3 || IMXRT_LPUART3

#if defined(HAS_KINETISK_UART4)
    case 4:
      sendHandler_ = std::make_unique<UARTSendHandler>(
          serialIndex_, this, &KINETISK_UART4, IRQ_UART4_STATUS, &uart4_tx_isr);
      break;
#elif defined(IMXRT_LPUART8)
    case 4:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART8, IRQ_LPUART8, lpuart8_tx_isr);
      break;
#endif  // HAS_KINETISK_UART4 || IMXRT_LPUART8

#if defined(HAS_KINETISK_UART5)
    case 5:
      sendHandler_ = std::make_unique<UARTSendHandler>(
          serialIndex_, this, &KINETISK_UART5, IRQ_UART5_STATUS, &uart5_tx_isr);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &KINETISK_LPUART0, IRQ_LPUART0, lpuart0_tx_isr);
      break;
#elif defined(IMXRT_LPUART1)
    case 5:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART1, IRQ_LPUART1, lpuart1_tx_isr);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0 || IMXRT_LPUART1

#if defined(IMXRT_LPUART7)
    case 6:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART7, IRQ_LPUART7, lpuart7_tx_isr);
      break;
#endif  // IMXRT_LPUART7

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))
    case 7:
      sendHandler_ = std::make_unique<LPUARTSendHandler>(
          serialIndex_, this, &IMXRT_LPUART5, IRQ_LPUART5, lpuart5_tx_isr);
      break;
#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)

    default:
      break;
  }
}

Sender::~Sender() {
  end();
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

  sendHandler_->start();
  intervalTimer_.setPriority(sendHandler_->priority());
  // Also set the interval timer priority to match the UART priority

  sendHandler_->setActive();
}

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

  sendHandler_->end();
  intervalTimer_.end();

  // Remove the reference from the instances,
  // but only if we're the ones who added it
  if (txInstances[serialIndex_] == this) {
    txInstances[serialIndex_] = nullptr;
  }
}

#ifndef TEENSYDMX_USE_PERIODICTIMER
void Sender::setBreakTime(uint32_t t) {
  breakTime_ = t;
  adjustedBreakTime_ = t + kBreakTimerAdjust;
}
#endif  // !TEENSYDMX_USE_PERIODICTIMER

uint32_t Sender::breakTime() const {
  if (isBreakUseTimerNotSerial()) {
    return breakTime_;
  }

  uint32_t t;
  switch (breakSerialFormat() &
          ~(kSerialFormatTXINVBit | kSerialFormatRXINVBit)) {
    case SERIAL_7E1:
    case SERIAL_8N1:
    case SERIAL_8O1:
    case SERIAL_8N2: t = 9; break;
    case SERIAL_8E1: t = 10; break;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL) || \
    defined(__IMXRT1062__) || defined(__IMXRT1052__)
    case SERIAL_8E2: t = 10; break;
    case SERIAL_8O2: t = 9; break;
#endif  // Serial 8E2- and 8O2-supporting chips
    case SERIAL_7O1: t = 8; break;
#ifdef SERIAL_9BIT_SUPPORT
    case SERIAL_9N1:
    case SERIAL_9O1: t = 10; break;
    case SERIAL_9E1: t = 11; break;
#endif  // SERIAL_9BIT_SUPPORT
    default:
      return kDefaultBreakTime;
  }

  return (t * 1000000) / breakSerialBaud();
}

void Sender::setMABTime(uint32_t t) {
  mabTime_ = t;
  if (t <= kMABTimerAdjust) {
    adjustedMABTime_ = 0;
  } else {
    adjustedMABTime_ = t - kMABTimerAdjust;
  }
}

uint32_t Sender::mabTime() const {
  if (isBreakUseTimerNotSerial()) {
    return mabTime_;
  }

  uint32_t t;
  switch (breakSerialFormat() &
          ~(kSerialFormatTXINVBit | kSerialFormatRXINVBit)) {
    case SERIAL_7E1:
    case SERIAL_8N1:
    case SERIAL_8E1: t = 1; break;
    case SERIAL_7O1:
    case SERIAL_8O1:
    case SERIAL_8N2: t = 2; break;
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL) || \
    defined(__IMXRT1062__) || defined(__IMXRT1052__)
    case SERIAL_8E2: t = 2; break;
    case SERIAL_8O2: t = 3; break;
#endif  // Serial 8E2- and 8O2-supporting chips
#ifdef SERIAL_9BIT_SUPPORT
    case SERIAL_9N1:
    case SERIAL_9E1: t = 1; break;
    case SERIAL_9O1: t = 2; break;
#endif  // SERIAL_9BIT_SUPPORT
    default:
      return kDefaultMABTime;
  }

  return (t * 1000000) / breakSerialBaud();
}

bool Sender::setBreakSerialParams(uint32_t baud, uint32_t format) {
  // Check the parameters
  if (baud == 0) {
    return false;
  }
  if ((format & kSerialFormatTXINVBit) != 0) {
    return false;
  }

  switch (breakSerialFormat() &
          ~(kSerialFormatTXINVBit | kSerialFormatRXINVBit)) {
    case SERIAL_7E1:
    case SERIAL_7O1:
    case SERIAL_8N1:
    case SERIAL_8E1:
    case SERIAL_8O1:
    case SERIAL_8N2:
#if defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(KINETISL) || \
    defined(__IMXRT1062__) || defined(__IMXRT1052__)
    case SERIAL_8E2:
    case SERIAL_8O2:
#endif  // Serial 8E2- and 8O2-supporting chips
#ifdef SERIAL_9BIT_SUPPORT
    case SERIAL_9N1:
    case SERIAL_9E1:
    case SERIAL_9O1:
#endif  // SERIAL_9BIT_SUPPORT
      break;

    default:
      return false;
  }

  breakBaud_ = baud;
  breakFormat_ = format;
  sendHandler_->breakSerialParamsChanged();
  return true;
}

void Sender::setInterSlotTime(uint32_t t) {
  interSlotTime_ = t;
  if (t <= kInterSlotTimerAdjust) {
    adjustedInterSlotTime_ = 0;
  } else {
    adjustedInterSlotTime_ = t - kInterSlotTimerAdjust;
  }
}

uint32_t Sender::interSlotTime() const {
  return interSlotTime_;
}

bool Sender::setPacketSizeAndData(int size,
                                  int startChannel,
                                  const uint8_t *values,
                                  int len) {
  if (size <= 0 || kMaxDMXPacketSize < size) {
    return false;
  }
  if (len < 0 || startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return false;
  }
  if (len == 0) {
    activePacketSize_ = size;
    return true;
  }
  if (startChannel + len <= 0 || kMaxDMXPacketSize < startChannel + len) {
    return false;
  }
  if (values == nullptr) {
    return false;
  }

  if (activePacketSize_ == size) {
    std::copy_n(&values[0], len, &activeBuf_[startChannel]);
  } else {
    Lock lock{*this};
    //{
      activePacketSize_ = size;
      std::copy_n(&values[0], len, &activeBuf_[startChannel]);
    //}
  }
  return true;
}

bool Sender::setPacketSize(int size) {
  if (size <= 0 || kMaxDMXPacketSize < size) {
    return false;
  }
  activePacketSize_ = size;
  return true;
}

bool Sender::set(int channel, uint8_t value) {
  if (channel < 0 || kMaxDMXPacketSize <= channel) {
    return false;
  }
  Lock lock{*this};
  //{
    activeBuf_[channel] = value;
  //}
  return true;
}

bool Sender::set16Bit(int channel, uint16_t value) {
  if (channel < 0 || kMaxDMXPacketSize - 1 <= channel) {
    return false;
  }

  Lock lock{*this};
  //{
    activeBuf_[channel] = value >> 8;
    activeBuf_[channel + 1] = value;
  //}
  return true;
}

bool Sender::set(int startChannel, const uint8_t *values, int len) {
  if (len < 0 || startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return false;
  }
  if (len == 0) {
    return true;
  }
  if (startChannel + len <= 0 || kMaxDMXPacketSize < startChannel + len) {
    return false;
  }
  if (values == nullptr) {
    return false;
  }

  Lock lock{*this};
  //{
    std::copy_n(&values[0], len, &activeBuf_[startChannel]);
  //}
  return true;
}

bool Sender::set16Bit(int startChannel, const uint16_t *values, int len) {
  if (len < 0 || startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return false;
  }
  if (len == 0) {
    return true;
  }
  if (startChannel + len*2 <= 0 || kMaxDMXPacketSize < startChannel + len*2) {
    return false;
  }
  if (values == nullptr) {
    return false;
  }

  Lock lock{*this};
  //{
    for (int i = 0; i < len; i++) {
      activeBuf_[startChannel++] = values[i] >> 8;
      activeBuf_[startChannel++] = values[i];
    }
  //}
  return true;
}

void Sender::clear() {
  Lock lock{*this};
  //{
    std::fill_n(&activeBuf_[0], kMaxDMXPacketSize, uint8_t{0});
  //}
}

bool Sender::fill(int startChannel, int len, uint8_t value) {
  if (len < 0 || startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return false;
  }
  if (len == 0) {
    return true;
  }
  if (startChannel + len <= 0 || kMaxDMXPacketSize < startChannel + len) {
    return false;
  }

  Lock lock{*this};
  //{
    std::fill_n(&activeBuf_[startChannel], len, value);
  //}
  return true;
}

void Sender::setMBBTime(uint32_t t) {
  mbbTime_ = t;
  if (t <= kMBBTimerMin) {
    adjustedMBBTime_ = 0;
  } else if (t <= kMBBTimerMinAdjusted + kMBBTimerAdjust) {
    adjustedMBBTime_ = kMBBTimerMinAdjusted;
  } else {
    adjustedMBBTime_ = t - kMBBTimerAdjust;
  }
}

uint32_t Sender::mbbTime() const {
  return mbbTime_;
}

bool Sender::setRefreshRate(float rate) {
  if ((rate != rate) || rate < 0.0f) {  // NaN or negative
    return false;
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
  return true;
}

void Sender::resume() {
  resumeFor(0);
}

bool Sender::resumeFor(int n) {
  return resumeFor(n, doneTXFunc_);
}

bool Sender::resumeFor(int n, void (*doneTXFunc)(Sender *s)) {
  if (n < 0) {
    return false;
  }

  // Pausing made transmission INACTIVE
  Lock lock{*this};
  //{
    resumeCounter_ = n;
    if (paused_) {
      // Copy the active buffer into the inactive buffer
      std::copy_n(&activeBuf_[0], kMaxDMXPacketSize, &inactiveBuf_[0]);
      inactivePacketSize_ = activePacketSize_;

      if (began_ && !transmitting_) {
        sendHandler_->setActive();
      }

      paused_ = false;
    }
    doneTXFunc_ = doneTXFunc;
  //}

  return true;
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
  // Copy the active buffer into the inactive buffer
  std::copy_n(&activeBuf_[0], kMaxDMXPacketSize, &inactiveBuf_[0]);
  inactivePacketSize_ = activePacketSize_;

  incPacketCount();
  inactiveBufIndex_ = 0;
  transmitting_ = false;
  state_ = XmitStates::kIdle;

  if (paused_) {
    void (*f)(Sender *) = doneTXFunc_;
    if (f != nullptr) {
      f(this);
    }
  }
}

// ---------------------------------------------------------------------------
//  IRQ management
// ---------------------------------------------------------------------------

void Sender::setIRQState(bool flag) const {
  if (!began_) {
    return;
  }
  sendHandler_->setIRQState(flag);
}

// ---------------------------------------------------------------------------
//  UART0 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)

void uart0_tx_isr() {
  Sender *s = txInstances[0];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

// ---------------------------------------------------------------------------
//  UART1 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)

void uart1_tx_isr() {
  Sender *s = txInstances[1];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

// ---------------------------------------------------------------------------
//  UART2 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)

void uart2_tx_isr() {
  Sender *s = txInstances[2];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

// ---------------------------------------------------------------------------
//  UART3 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART3)

void uart3_tx_isr() {
  Sender *s = txInstances[3];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART4)

void uart4_tx_isr() {
  Sender *s = txInstances[4];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 TX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART5)

void uart5_tx_isr() {
  Sender *s = txInstances[5];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 TX ISR (Serial6 on Teensy 3.6)
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_LPUART0)

void lpuart0_tx_isr() {
  Sender *s = txInstances[5];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART0

// ---------------------------------------------------------------------------
//  LPUART6 TX ISR (Serial1 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART6)

void lpuart6_tx_isr() {
  Sender *s = txInstances[0];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART6

// ---------------------------------------------------------------------------
//  LPUART4 TX ISR (Serial2 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART4)

void lpuart4_tx_isr() {
  Sender *s = txInstances[1];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART4

// ---------------------------------------------------------------------------
//  LPUART2 TX ISR (Serial3 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART2)

void lpuart2_tx_isr() {
  Sender *s = txInstances[2];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART2

// ---------------------------------------------------------------------------
//  LPUART3 TX ISR (Serial4 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART3)

void lpuart3_tx_isr() {
  Sender *s = txInstances[3];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART3

// ---------------------------------------------------------------------------
//  LPUART8 TX ISR (Serial5 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART8)

void lpuart8_tx_isr() {
  Sender *s = txInstances[4];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART8

// ---------------------------------------------------------------------------
//  LPUART1 TX ISR (Serial6 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART1)

void lpuart1_tx_isr() {
  Sender *s = txInstances[5];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART1

// ---------------------------------------------------------------------------
//  LPUART7 TX ISR (Serial7 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART7)

void lpuart7_tx_isr() {
  Sender *s = txInstances[6];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART7

// ---------------------------------------------------------------------------
//  LPUART5 TX ISR (Serial8 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))

void lpuart5_tx_isr() {
  Sender *s = txInstances[7];
  if (s != nullptr) {
    s->sendHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)

// Undefine these macros
#undef UART_C2_TX_ENABLE
#undef UART_C2_TX_ACTIVE
#undef UART_C2_TX_COMPLETING
#undef UART_C2_TX_INACTIVE
#undef LPUART_CTRL_TX_ENABLE
#undef LPUART_CTRL_TX_ACTIVE
#undef LPUART_CTRL_TX_COMPLETING
#undef LPUART_CTRL_TX_INACTIVE
#undef ACTIVATE_UART_TX_SERIAL
#undef ACTIVATE_LPUART_TX_SERIAL

}  // namespace teensydmx
}  // namespace qindesign
