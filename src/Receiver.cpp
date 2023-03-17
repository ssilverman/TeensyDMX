// This file is part of the TeensyDMX library.
// (c) 2017-2023 Shawn Silverman

#include "TeensyDMX.h"

// C++ includes
#include <algorithm>
#include <atomic>
#include <utility>

#include <core_pins.h>
#include <util/atomic.h>

#include "Responder.h"

namespace qindesign {
namespace teensydmx {

extern const uint32_t kCharTime;  // In microseconds
const uint32_t kCharTimeLow = 0.98f * kCharTime;

constexpr uint32_t kMinBreakTime = 88;  // In microseconds
constexpr uint32_t kMinMABTime   = 8;   // In microseconds

// Routines:
// 1. RX ISR routines, and
// 2. Routines that do raw transmit.
//    These don't affect the transmitter.
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
void uart0_rx_isr();
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
void uart1_rx_isr();
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
void uart2_rx_isr();
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
void uart3_rx_isr();
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
void uart4_rx_isr();
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
void uart5_rx_isr();
#endif  // HAS_KINETISK_UART5

#if defined(HAS_KINETISK_LPUART0)
void lpuart0_rx_isr();
#endif  // HAS_KINETISK_LPUART0

#if defined(IMXRT_LPUART6)
void lpuart6_rx_isr();
#endif  // IMXRT_LPUART6

#if defined(IMXRT_LPUART4)
void lpuart4_rx_isr();
#endif  // IMXRT_LPUART4

#if defined(IMXRT_LPUART2)
void lpuart2_rx_isr();
#endif  // IMXRT_LPUART2

#if defined(IMXRT_LPUART3)
void lpuart3_rx_isr();
#endif  // IMXRT_LPUART3

#if defined(IMXRT_LPUART8)
void lpuart8_rx_isr();
#endif  // IMXRT_LPUART8

#if defined(IMXRT_LPUART1)
void lpuart1_rx_isr();
#endif  // IMXRT_LPUART1

#if defined(IMXRT_LPUART7)
void lpuart7_rx_isr();
#endif  // IMXRT_LPUART7

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))
void lpuart5_rx_isr();
#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)

// Used by the RX ISRs.
#if defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41)
static Receiver *volatile rxInstances[8]{nullptr};
#else
static Receiver *volatile rxInstances[7]{nullptr};
#endif  // __IMXRT1052__ || ARDUINO_TEENSY41

// Forward declarations of RX watch pin ISRs
void rxPinFellSerial0_isr();
void rxPinRoseSerial0_isr();
void rxPinFellSerial1_isr();
void rxPinRoseSerial1_isr();
void rxPinFellSerial2_isr();
void rxPinRoseSerial2_isr();
void rxPinFellSerial3_isr();
void rxPinRoseSerial3_isr();
void rxPinFellSerial4_isr();
void rxPinRoseSerial4_isr();
void rxPinFellSerial5_isr();
void rxPinRoseSerial5_isr();
void rxPinFellSerial6_isr();
void rxPinRoseSerial6_isr();
#if defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41)
void rxPinFellSerial7_isr();
void rxPinRoseSerial7_isr();
#endif  // __IMXRT1052__ || ARDUINO_TEENSY41

#if defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41)
// RX watch pin fell ISRs.
static void (*rxPinFellISRs[8])() {
    rxPinFellSerial0_isr,
    rxPinFellSerial1_isr,
    rxPinFellSerial2_isr,
    rxPinFellSerial3_isr,
    rxPinFellSerial4_isr,
    rxPinFellSerial5_isr,
    rxPinFellSerial6_isr,
    rxPinFellSerial7_isr,
};
// RX watch pin rose ISRs.
static void (*rxPinRoseISRs[8])() {
    rxPinRoseSerial0_isr,
    rxPinRoseSerial1_isr,
    rxPinRoseSerial2_isr,
    rxPinRoseSerial3_isr,
    rxPinRoseSerial4_isr,
    rxPinRoseSerial5_isr,
    rxPinRoseSerial6_isr,
    rxPinRoseSerial7_isr,
};
#else
// RX watch pin fell ISRs.
static void (*rxPinFellISRs[7])() {
    rxPinFellSerial0_isr,
    rxPinFellSerial1_isr,
    rxPinFellSerial2_isr,
    rxPinFellSerial3_isr,
    rxPinFellSerial4_isr,
    rxPinFellSerial5_isr,
    rxPinFellSerial6_isr,
};
// RX watch pin rose ISRs.
static void (*rxPinRoseISRs[7])() {
    rxPinRoseSerial0_isr,
    rxPinRoseSerial1_isr,
    rxPinRoseSerial2_isr,
    rxPinRoseSerial3_isr,
    rxPinRoseSerial4_isr,
    rxPinRoseSerial5_isr,
    rxPinRoseSerial6_isr,
};
#endif  // __IMXRT1052__ || ARDUINO_TEENSY41

Receiver::Receiver(HardwareSerial &uart)
    : TeensyDMX(uart),
      txEnabled_(true),
      began_(false),
      state_{RecvStates::kIdle},
      keepShortPackets_(false),
      buf1_{0},
      buf2_{0},
      activeBuf_(buf1_),
      inactiveBuf_(buf2_),
      activeBufIndex_(0),
      packetSize_(0),
      lastBreakStartTime_(0),
      breakStartTime_(0),
      lastSlotEndTime_(0),
      connected_(false),
      connectChangeFunc_{nullptr},
      responderCount_(0),
      responderOutBufLen_(0),
      setTXNotRXFunc_(nullptr),
      rxWatchPin_(-1),
      seenMABStart_(false),
      seenMABEnd_(false),
      mabStartTime_(0),
      mabEndTime_(0) {
  switch(serialIndex_) {
#if defined(HAS_KINETISK_UART0)
    case 0:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART0, IRQ_UART0_STATUS,
          IRQ_UART0_ERROR, &uart0_rx_isr);
      break;
#elif defined(HAS_KINETISL_UART0)
    case 0:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART0, IRQ_UART0_STATUS, &uart0_rx_isr);
      break;
#elif defined(IMXRT_LPUART6)
    case 0:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART6, IRQ_LPUART6, &lpuart6_rx_isr);
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0 || IMXRT_LPUART6

#if defined(HAS_KINETISK_UART1)
    case 1:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART1, IRQ_UART1_STATUS,
          IRQ_UART1_ERROR, &uart1_rx_isr);
      break;
#elif defined(HAS_KINETISL_UART1)
    case 1:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART1, IRQ_UART1_STATUS, &uart1_rx_isr);
      break;
#elif defined(IMXRT_LPUART4)
    case 1:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART4, IRQ_LPUART4, &lpuart4_rx_isr);
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1 || IMXRT_LPUART4

#if defined(HAS_KINETISK_UART2)
    case 2:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART2, IRQ_UART2_STATUS,
          IRQ_UART2_ERROR, &uart2_rx_isr);
      break;
#elif defined(HAS_KINETISL_UART2)
    case 2:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART2, IRQ_UART2_STATUS, &uart2_rx_isr);
      break;
#elif defined(IMXRT_LPUART2)
    case 2:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART2, IRQ_LPUART2, &lpuart2_rx_isr);
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2 || IMXRT_LPUART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART3, IRQ_UART3_STATUS,
          IRQ_UART3_ERROR, &uart3_rx_isr);
      break;
#elif defined(IMXRT_LPUART3)
    case 3:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART3, IRQ_LPUART3, &lpuart3_rx_isr);
      break;
#endif  // HAS_KINETISK_UART3 || IMXRT_LPUART3

#if defined(HAS_KINETISK_UART4)
    case 4:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART4, IRQ_UART4_STATUS,
          IRQ_UART4_ERROR, &uart4_rx_isr);
      break;
#elif defined(IMXRT_LPUART8)
    case 4:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART8, IRQ_LPUART8, &lpuart8_rx_isr);
      break;
#endif  // HAS_KINETISK_UART4 || IMXRT_LPUART8

#if defined(HAS_KINETISK_UART5)
    case 5:
      receiveHandler_ = std::make_unique<UARTReceiveHandler>(
          serialIndex_, this, &KINETISK_UART5, IRQ_UART5_STATUS,
          IRQ_UART5_ERROR, &uart5_rx_isr);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &KINETISK_LPUART0, IRQ_LPUART0, &lpuart0_rx_isr);
      break;
#elif defined(IMXRT_LPUART1)
    case 5:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART1, IRQ_LPUART1, &lpuart1_rx_isr);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0 || IMXRT_LPUART1

#if defined(IMXRT_LPUART7)
    case 6:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART7, IRQ_LPUART7, &lpuart7_rx_isr);
      break;
#endif  // IMXRT_LPUART7

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))
    case 7:
      receiveHandler_ = std::make_unique<LPUARTReceiveHandler>(
          serialIndex_, this, &IMXRT_LPUART5, IRQ_LPUART5, &lpuart5_rx_isr);
      break;
#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)

    default:
      break;
  }
}

Receiver::~Receiver() {
  end();
}

void Receiver::setTXEnabled(bool flag) {
  if (txEnabled_ == flag) {
    return;
  }

  txEnabled_ = flag;
  if (!began_) {
    return;
  }

  receiveHandler_->setTXEnabled(flag);
}

void Receiver::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  if (serialIndex_ < 0) {
    return;
  }

  // Reset all the stats
  resetPacketCount();
  packetSize_ = 0;
  lastBreakStartTime_ = 0;
  packetStats_ = PacketStats{};
  errorStats_ = ErrorStats{};

  // Set up the instance for the ISRs
  Receiver *r = rxInstances[serialIndex_];
  rxInstances[serialIndex_] = this;
  if (r != nullptr && r != this) {  // NOTE: Shouldn't be able to be 'this'
    r->end();
  }

  state_ = RecvStates::kIdle;
  activeBufIndex_ = 0;
  setConnected(false);

  receiveHandler_->start();
  intervalTimer_.setPriority(receiveHandler_->priority());
  // Also set the interval timer priority to match the UART priority

  // Enable receive
  setTXNotRX(false);
}

void Receiver::end() {
  if (!began_) {
    return;
  }
  began_ = false;

  if (serialIndex_ < 0) {
    return;
  }

  // Remove any chance that our RX ISRs start after end() is called,
  // so disable the IRQs first

  receiveHandler_->end();

  // Remove the reference from the instances,
  // but only if we're the ones who added it
  if (rxInstances[serialIndex_] == this) {
    rxInstances[serialIndex_] = nullptr;
  }

  setConnected(false);
}

int Receiver::readPacket(uint8_t *buf, int startChannel, int len,
                         PacketStats *stats) {
  if (len <= 0 || startChannel < 0 || kMaxDMXPacketSize <= startChannel) {
    return 0;
  }

  int retval = -1;
  Lock lock{*this};
  //{
    // No need to poll for a timeout here because IDLE detection
    // handles this now

    if (packetSize_ > 0) {
      if (startChannel >= packetSize_) {
        retval = 0;
      } else {
        if (startChannel + len > packetSize_) {
          len = packetSize_ - startChannel;
        }
        std::copy_n(&inactiveBuf_[startChannel], len, &buf[0]);
        retval = len;
      }
      packetSize_ = 0;
    }
    if (stats != nullptr) {
      std::atomic_signal_fence(std::memory_order_acquire);
      *stats = packetStats_;
    }
  //}
  return retval;
}

uint8_t Receiver::get(int channel, bool *rangeError) const {
  if (rangeError != nullptr) {
    *rangeError = true;
  }
  if (channel < 0 || kMaxDMXPacketSize <= channel) {
    return 0;
  }

  uint8_t b = 0;
  Lock lock{*this};
  //{
    // Since channel >= 0, lastPacketSize_ > channel implies lastPacketSize_ > 0
    std::atomic_signal_fence(std::memory_order_acquire);
    if (channel < packetStats_.size) {
      if (rangeError != nullptr) {
        *rangeError = false;
      }
      b = inactiveBuf_[channel];
    }
  //}
  return b;
}

uint16_t Receiver::get16Bit(int channel, bool *rangeError) const {
  if (rangeError != nullptr) {
    *rangeError = true;
  }
  if (channel < 0 || kMaxDMXPacketSize - 1 <= channel) {
    return 0;
  }

  uint16_t v = 0;
  Lock lock{*this};
  //{
    // Since channel >= 0, lastPacketSize_ - 1 > channel
    // implies lastPacketSize_ - 1 > 0
    std::atomic_signal_fence(std::memory_order_acquire);
    if (channel < packetStats_.size - 1) {
      if (rangeError != nullptr) {
        *rangeError = false;
      }
      v = (uint16_t{inactiveBuf_[channel]} << 8) |
          uint16_t{inactiveBuf_[channel + 1]};
    }
  //}
  return v;
}

Receiver::PacketStats Receiver::packetStats() const {
  Lock lock{*this};
  std::atomic_signal_fence(std::memory_order_acquire);
  return packetStats_;
}

uint32_t Receiver::lastPacketTimestamp() const {
  Lock lock{*this};
  std::atomic_signal_fence(std::memory_order_acquire);
  return packetStats_.timestamp;
}

Receiver::ErrorStats Receiver::errorStats() const {
  Lock lock{*this};
  std::atomic_signal_fence(std::memory_order_acquire);
  return errorStats_;
}

Responder *Receiver::setResponder(uint8_t startCode, Responder *r) {
  // For a null responder, delete any current one for this start code
  if (r == nullptr) {
    Lock lock{*this};

    if (responders_ == nullptr) {
      return nullptr;
    }

    // Replace any previous responder
    Responder *old = responders_[startCode];
    if (old != nullptr) {
      responderCount_--;
    }

    // When no more responders, delete all the buffers
    if (responderCount_ == 0) {
      responderOutBuf_ = nullptr;
      responders_ = nullptr;
    }

    return old;
  }

  Lock lock{*this};

  // Allocate this first because it's done once. The output buffer may get
  // reallocated, and so letting that be the last thing deleted avoids
  // potential fragmentation.
  if (responders_ == nullptr) {
    responders_.reset(new Responder *[256]);
    // Allocation may have failed on small systems
    if (responders_ == nullptr) {
      return nullptr;
    }
  }

  // Initialize the output buffer
  int outBufSize = r->outputBufferSize();
  if (responderOutBuf_ == nullptr || responderOutBufLen_ < outBufSize) {
    responderOutBuf_.reset(new uint8_t[outBufSize]);
    // Allocation may have failed on small systems
    if (responderOutBuf_ == nullptr) {
      responderOutBufLen_ = 0;
      responders_ = nullptr;
      responderCount_ = 0;
      return nullptr;
    }
    responderOutBufLen_ = outBufSize;
  }

  // If a responder is already set then the output buffer should be the
  // correct size
  Responder *old = responders_[startCode];
  responders_[startCode] = r;
  if (old == nullptr) {
    responderCount_++;
  }

  return old;
}

void Receiver::completePacket(RecvStates newState) {
  uint32_t t = millis();
  state_ = newState;  // Should only be kIdle or kDataIdle

  receiveHandler_->setILT(false);  // Set IDLE detection to "after start bit"

  // An empty packet isn't valid, there must be at least a start code
  if (activeBufIndex_ <= 0) {
    // kDataIdle isn't used unless the packet is full, so don't need to
    // reset to kIdle here
    return;
  }

  // Check for a short packet. If found, discard the data if the "keep short
  // packets" feature is disabled; otherwise, don't discard the data but mark it
  // as "short".
  // Do this check after first checking activeBufIndex_ because a positive value
  // means that the following start and end time variables are valid
  if (lastSlotEndTime_ - breakStartTime_ < kMinDMXPacketTime) {
    errorStats_.shortPacketCount++;
    if (keepShortPackets_) {
      packetStats_.isShort = true;
    } else {
      packetStats_.isShort = false;
      activeBufIndex_ = 0;
    }
  } else {
    packetStats_.isShort = false;
  }

  // Swap the buffers
  if (activeBuf_ == buf1_) {
    activeBuf_ = buf2_;
    inactiveBuf_ = buf1_;
  } else {
    activeBuf_ = buf1_;
    inactiveBuf_ = buf2_;
  }

  incPacketCount();

  // Packet stats
  packetStats_.size = packetSize_ = activeBufIndex_;
  packetStats_.extraSize = 0;
  packetStats_.timestamp = t;
  packetStats_.frameTimestamp = breakStartTime_;
  packetStats_.packetTime = lastSlotEndTime_ - breakStartTime_;
  packetStats_.breakPlusMABTime = packetStats_.nextBreakPlusMABTime;
  packetStats_.breakTime = packetStats_.nextBreakTime;
  packetStats_.mabTime = packetStats_.nextMABTime;

  // Let the responder, if any, process the packet
  if (responders_ != nullptr) {
    Responder *r = responders_[inactiveBuf_[0]];
    if (r != nullptr) {
      r->receivePacket(inactiveBuf_, packetSize_);
      if (r->eatPacket()) {
        packetStats_.extraSize = packetStats_.size = packetSize_ = 0;
      }
    }
  }
  std::atomic_signal_fence(std::memory_order_release);

  activeBufIndex_ = 0;
}

void Receiver::idleTimerCallback() {
  intervalTimer_.end();
  completePacket(RecvStates::kIdle);
  setConnected(false);
}

void Receiver::receiveIdle(uint32_t eventTime) {
  switch (state_) {
    case RecvStates::kBreak:
      if (seenMABStart_) {
        if (!seenMABEnd_) {
          if (rxWatchPin_ >= 0) {
            detachInterrupt(rxWatchPin_);
          }
        }
        if ((mabStartTime_ - breakStartTime_) < kMinBreakTime) {
          seenMABStart_ = false;
          receiveBadBreak();
          return;
        }
      } else {
        // This catches the case where a short BREAK is followed by a longer MAB
        if ((eventTime - breakStartTime_) < kMinBreakTime + kCharTimeLow) {
          seenMABStart_ = false;
          receiveBadBreak();
          return;
        }

        // We can infer what the rise time is here
        seenMABStart_ = true;
        mabStartTime_ = eventTime - kCharTime;
        receiveHandler_->setILT(true);  // IDLE detection to "after stop bit"
      }
      break;

    case RecvStates::kData:
      if ((eventTime - breakStartTime_) > kMaxDMXPacketTime ||
          (eventTime - lastSlotEndTime_) >= kMaxDMXIdleTime) {
        // We'll consider this as a packet end and not as a timeout
        // errorStats_.packetTimeoutCount++;
        completePacket(RecvStates::kIdle);
        setConnected(false);
        return;
      }
      break;

    case RecvStates::kDataIdle:
      state_ = RecvStates::kIdle;
      break;

    default:
      break;
  }

  // Start a timer watching for disconnection/packet end
  intervalTimer_.begin([this]() { idleTimerCallback(); },
                       kMaxDMXIdleTime - kCharTime);
}

void Receiver::receivePotentialBreak(uint32_t eventTime) {
  intervalTimer_.end();

  // A potential BREAK is detected when a stop bit is expected but not
  // received, and this happens after the start bit, nine bits, and the
  // missing stop bit, about 44us.
  // Note that breakStartTime_ only represents a potential BREAK start
  // time until we receive the first character.
  breakStartTime_ = eventTime - kCharTime;

  state_ = RecvStates::kBreak;

  // At this point, we don't know whether to keep or discard any collected
  // data because the BREAK may be invalid. In other words, don't make any
  // framing error or short packet decisions until we know the nature of
  // this BREAK.

  if (rxWatchPin_ >= 0) {
    seenMABStart_ = false;
    attachInterrupt(rxWatchPin_, rxPinRoseISRs[serialIndex_], RISING);
  }
}

void Receiver::receiveBadBreak() {
  intervalTimer_.end();

  // Not a BREAK
  errorStats_.framingErrorCount++;
  std::atomic_signal_fence(std::memory_order_release);

  // Don't keep the packet
  // See: [BREAK timing at the receiver](http://www.rdmprotocol.org/forums/showthread.php?t=1292)
  activeBufIndex_ = 0;
  completePacket(RecvStates::kIdle);

  // Consider this case as not seeing a BREAK
  // This may be line noise, so now we can't tell for sure where the
  // last BREAK was
  setConnected(false);
}

void Receiver::receiveByte(uint8_t b, uint32_t eopTime) {
  intervalTimer_.end();

  // Bad BREAKs are detected when BREAK + MAB + character time is too short
  // BREAK: 88us
  // MAB: 8us
  // Character time: 44us

  switch (state_) {
    case RecvStates::kBreak: {
      // BREAK and MAB timing check
      // Measure the BREAK and MAB, but don't set until after a
      // potential completePacket()
      uint32_t breakTime = 0;
      uint32_t mabTime = 0;
      if (seenMABStart_) {
        seenMABStart_ = false;
        if (seenMABEnd_) {
          mabTime = mabEndTime_ - mabStartTime_;
          if ((mabStartTime_ - breakStartTime_ < kMinBreakTime) ||
              (mabTime < kMinMABTime)) {
            receiveBadBreak();
            return;
          }
        } else {
          if (rxWatchPin_ >= 0) {
            detachInterrupt(rxWatchPin_);
          }
          if ((mabStartTime_ - breakStartTime_ < kMinBreakTime) ||
              (eopTime - mabStartTime_ < kMinMABTime + kCharTimeLow)) {
            receiveBadBreak();
            return;
          }
          mabTime = eopTime - kCharTime - mabStartTime_;
        }
        breakTime = mabStartTime_ - breakStartTime_;
        if (mabTime >= kMaxDMXIdleTime) {
          completePacket(RecvStates::kIdle);
          setConnected(false);
          return;
        }
      } else {
        // This is only a rudimentary check for short BREAKs. It does not
        // detect short BREAKs followed by long MABs. It only detects
        // whether BREAK + MAB time is at least 88us + 8us.
        if ((eopTime - breakStartTime_) <
            kMinBreakTime + kMinMABTime + kCharTimeLow) {
          // First byte is too early, discard any data
          receiveBadBreak();
          return;
        }
        receiveHandler_->setILT(true);  // IDLE detection to "after stop bit"
        // Since we've already received a byte, the idle detection can't start
        // again until the end of the next byte not already in the buffer
      }

      if (connected_) {  // This condition indicates we haven't seen some
                         // timeout and that lastBreakStartTime_ is valid
        // Complete any un-flushed bytes
        uint32_t dt = breakStartTime_ - lastBreakStartTime_;
        packetStats_.breakToBreakTime = dt;

        // In the following checks, the packet time limits are the same as the
        // BREAK-to-BREAK time limits
        if (dt < kMinDMXPacketTime) {
          errorStats_.shortPacketCount++;
          // Discard the data
          activeBufIndex_ = 0;
        } else if (dt > kMaxDMXPacketTime) {
          // NOTE: Zero-length packets will also trigger a timeout
          errorStats_.packetTimeoutCount++;
          // Keep the data
          // Don't disconnect because the timeout was relative to the
          // previous packet
        }
        completePacket(RecvStates::kIdle);
      } else {
        packetStats_.breakToBreakTime = 0;
        activeBufIndex_ = 0;
      }

      // Packet BREAK and MAB measurements
      // Store 'next' values because packets aren't completed until the
      // following BREAK (or timeout or size limit) and we need the
      // previous values
      packetStats_.nextBreakPlusMABTime = eopTime - kCharTime - breakStartTime_;
      packetStats_.nextBreakTime = breakTime;
      packetStats_.nextMABTime = mabTime;

      lastBreakStartTime_ = breakStartTime_;
      setConnected(true);
      state_ = RecvStates::kData;
      break;
    }

    case RecvStates::kData: {
      if (activeBufIndex_ == 0) {
        // OLD NOTE:
        // Checking this here accounts for buffered input, where several
        // bytes come in at the same time
        // OLD: uint32_t charTime = kCharTimeLow*(1 + activeBufIndex_);
        if (eopTime - breakStartTime_ <
            kMinBreakTime + kMinMABTime + kCharTimeLow) {
          // First byte is too early, discard any data
          receiveBadBreak();
          return;
        }
      }
      // NOTE: Don't need to check for inter-slot MARK time being
      //       too large because the IDLE detection will catch that
      break;
    }

    case RecvStates::kDataIdle:
      // Conditions for recognizing an extra byte:
      // 1. Still within the minimum packet time; we have to cut it off
      //    somewhere, and this seems like a good point, and
      // 2. A responder hasn't eaten the packet. If a responder doesn't eat
      //    the packet then the packet size won't have been set to zero.
      if (eopTime - breakStartTime_ <= kMaxDMXPacketTime &&
          packetStats_.size > 0) {
        // If a responder cut the packet off early, then the processed
        // packet size may be < 513, so use the sum and not just the
        // extra size
        int size = packetStats_.size + packetStats_.extraSize;
        if (size == kMaxDMXPacketSize) {
          errorStats_.longPacketCount++;
        }
        packetStats_.extraSize++;
      } else {
        state_ = RecvStates::kIdle;
      }
      std::atomic_signal_fence(std::memory_order_release);
      return;

    case RecvStates::kIdle:
      return;

    default:
      // Ignore any extra bytes in a packet or any bytes outside a packet
      return;
  }

  // Check the timing and if we are out of range then complete any bytes
  // until, but not including, this one
  lastSlotEndTime_ = eopTime;
  if ((eopTime - breakStartTime_) > kMaxDMXPacketTime) {
    errorStats_.packetTimeoutCount++;
    std::atomic_signal_fence(std::memory_order_release);
    completePacket(RecvStates::kIdle);
    setConnected(false);
    return;
  }
  std::atomic_signal_fence(std::memory_order_release);

  bool packetFull = false;  // Indicates whether the maximum packet size
                            // has been reached.
                            // Using this is necessary so that the responder's
                            // processByte is called before its receivePacket.
  activeBuf_[activeBufIndex_++] = b;
  if (activeBufIndex_ == kMaxDMXPacketSize) {
    packetFull = true;
  }

  // See if a responder needs to process the byte and respond
  Responder *r = nullptr;
  if (responders_ != nullptr) {
    r = responders_[activeBuf_[0]];
  }
  if (r == nullptr) {
    if (packetFull) {
      completePacket(RecvStates::kDataIdle);
    }
    return;
  }

  // Let the responder process the data
  int respLen =
      r->processByte(activeBuf_, activeBufIndex_, responderOutBuf_.get());
  if (respLen <= 0) {
    if (packetFull) {
      // If the responder isn't done by now, it's too late for this packet
      // because the maximum packet size has been reached
      completePacket(RecvStates::kDataIdle);
    }
    return;
  }
  completePacket(RecvStates::kDataIdle);  // This is probably the best option,
                                          // even though there may be more bytes
  if (!txEnabled_) {
    return;
  }

  // Do the response
  if (r->isSendBreakForLastPacket()) {
    uint32_t delay = r->preBreakDelay();
    uint32_t dt = micros() - eopTime;
    if (dt < delay) {
      delayMicroseconds(delay - dt);
    }

    setTXNotRX(true);
    delay = r->preDataDelay();
    if (delay > 0) {
      delayMicroseconds(delay);
    }
    receiveHandler_->txBreak(r->breakTime(), r->mabTime());
  } else {
    uint32_t delay = r->preNoBreakDelay();
    uint32_t dt = micros() - eopTime;
    if (dt < delay) {
      delayMicroseconds(delay - dt);
    }

    setTXNotRX(true);
    delay = r->preDataDelay();
    if (delay > 0) {
      delayMicroseconds(delay);
    }
  }
  receiveHandler_->txData(responderOutBuf_.get(), respLen);
  setTXNotRX(false);
}

void Receiver::setConnected(bool flag) {
  if (connected_ != flag) {
    connected_ = flag;
    void (*f)(Receiver *r) = connectChangeFunc_;
    if (f != nullptr) {
      f(this);
    }
  }
}

// ---------------------------------------------------------------------------
//  IRQ management
// ---------------------------------------------------------------------------

void Receiver::setIRQState(bool flag) const {
  if (!began_) {
    return;
  }
  receiveHandler_->setIRQState(flag);
}

// ---------------------------------------------------------------------------
//  RX pin interrupt and ISRs
// ---------------------------------------------------------------------------

void Receiver::setRXWatchPin(int pin) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (pin < 0) {
      if (rxWatchPin_ >= 0) {
        detachInterrupt(rxWatchPin_);
      }
      rxWatchPin_ = -1;
      seenMABStart_ = false;
    } else {
      if (rxWatchPin_ != pin) {
        if (rxWatchPin_ >= 0) {
          detachInterrupt(rxWatchPin_);
        }
        rxWatchPin_ = pin;
        seenMABStart_ = false;
      }
    }
  }
}

void Receiver::rxPinFell_isr() {
  if (seenMABStart_) {
    mabEndTime_ = micros();
    seenMABEnd_ = true;
  }
  detachInterrupt(rxWatchPin_);
}

void Receiver::rxPinRose_isr() {
  if (!seenMABStart_) {
    mabStartTime_ = micros();
    seenMABStart_ = true;
    seenMABEnd_ = false;
    attachInterrupt(rxWatchPin_, rxPinFellISRs[serialIndex_], FALLING);
  } else {
    seenMABStart_ = false;
  }
  receiveHandler_->setILT(true);  // Set IDLE detection to "after stop bit"
}

void rxPinFellSerial0_isr() {
  Receiver *r = rxInstances[0];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial0_isr() {
  Receiver *r = rxInstances[0];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}

void rxPinFellSerial1_isr() {
  Receiver *r = rxInstances[1];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial1_isr() {
  Receiver *r = rxInstances[1];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}

void rxPinFellSerial2_isr() {
  Receiver *r = rxInstances[2];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial2_isr() {
  Receiver *r = rxInstances[2];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}

void rxPinFellSerial3_isr() {
  Receiver *r = rxInstances[3];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial3_isr() {
  Receiver *r = rxInstances[3];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}

void rxPinFellSerial4_isr() {
  Receiver *r = rxInstances[4];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial4_isr() {
  Receiver *r = rxInstances[4];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}

void rxPinFellSerial5_isr() {
  Receiver *r = rxInstances[5];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial5_isr() {
  Receiver *r = rxInstances[5];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}

void rxPinFellSerial6_isr() {
  Receiver *r = rxInstances[6];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial6_isr() {
  Receiver *r = rxInstances[6];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}

#if defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41)
void rxPinFellSerial7_isr() {
  Receiver *r = rxInstances[7];
  if (r != nullptr) {
    r->rxPinFell_isr();
  }
}

void rxPinRoseSerial7_isr() {
  Receiver *r = rxInstances[7];
  if (r != nullptr) {
    r->rxPinRose_isr();
  }
}
#endif  // __IMXRT1052__ || ARDUINO_TEENSY41

// ---------------------------------------------------------------------------
//  UART0 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)

void uart0_rx_isr() {
  Receiver *r = rxInstances[0];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

// ---------------------------------------------------------------------------
//  UART1 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)

void uart1_rx_isr() {
  Receiver *r = rxInstances[1];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

// ---------------------------------------------------------------------------
//  UART2 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)

void uart2_rx_isr() {
  Receiver *r = rxInstances[2];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

// ---------------------------------------------------------------------------
//  UART3 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART3)

void uart3_rx_isr() {
  Receiver *r = rxInstances[3];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART4)

void uart4_rx_isr() {
  Receiver *r = rxInstances[4];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART5)

void uart5_rx_isr() {
  Receiver *r = rxInstances[5];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 RX ISR (Serial6 on Teensy 3.6)
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_LPUART0)

void lpuart0_rx_isr() {
  Receiver *r = rxInstances[5];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // HAS_KINETISK_LPUART0

// ---------------------------------------------------------------------------
//  LPUART6 RX ISR (Serial1 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART6)

void lpuart6_rx_isr() {
  Receiver *r = rxInstances[0];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART6

// ---------------------------------------------------------------------------
//  LPUART4 RX ISR (Serial2 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART4)

void lpuart4_rx_isr() {
  Receiver *r = rxInstances[1];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART4

// ---------------------------------------------------------------------------
//  LPUART2 RX ISR (Serial3 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART2)

void lpuart2_rx_isr() {
  Receiver *r = rxInstances[2];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART2

// ---------------------------------------------------------------------------
//  LPUART3 RX ISR (Serial4 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART3)

void lpuart3_rx_isr() {
  Receiver *r = rxInstances[3];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART3

// ---------------------------------------------------------------------------
//  LPUART8 RX ISR (Serial5 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART8)

void lpuart8_rx_isr() {
  Receiver *r = rxInstances[4];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART8

// ---------------------------------------------------------------------------
//  LPUART1 RX ISR (Serial6 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART1)

void lpuart1_rx_isr() {
  Receiver *r = rxInstances[5];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART1

// ---------------------------------------------------------------------------
//  LPUART7 RX ISR (Serial7 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART7)

void lpuart7_rx_isr() {
  Receiver *r = rxInstances[6];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART7

// ---------------------------------------------------------------------------
//  LPUART5 RX ISR (Serial8 on Teensy 4)
// ---------------------------------------------------------------------------

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))

void lpuart5_rx_isr() {
  Receiver *r = rxInstances[7];
  if (r != nullptr) {
    r->receiveHandler_->irqHandler();
  }
}

#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)

}  // namespace teensydmx
}  // namespace qindesign
