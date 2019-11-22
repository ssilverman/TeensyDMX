// This file is part of the TeensyDMX library.
// (c) 2017-2019 Shawn Silverman

#include "TeensyDMX.h"

// C++ includes
#include <cstring>
#include <utility>

// Project includes
#include "Responder.h"
#include "uart_routine_defines.h"

namespace qindesign {
namespace teensydmx {

constexpr uint32_t kSlotsBaud   = 250000;                // 4us
constexpr uint32_t kSlotsFormat = SERIAL_8N2;            // 9:2
constexpr uint32_t kBitTime     = 1000000 / kSlotsBaud;  // In microseconds
constexpr uint32_t kCharTime    = 11 * kBitTime;         // In microseconds

// Routines that do raw transmit
// These don't affect the transmitter
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
void uart0_tx(const uint8_t *b, int len);
void uart0_tx_break(int count, uint32_t mabTime);
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
void uart1_tx(const uint8_t *b, int len);
void uart1_tx_break(int count, uint32_t mabTime);
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
void uart2_tx(const uint8_t *b, int len);
void uart2_tx_break(int count, uint32_t mabTime);
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
void uart3_tx(const uint8_t *b, int len);
void uart3_tx_break(int count, uint32_t mabTime);
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
void uart4_tx(const uint8_t *b, int len);
void uart4_tx_break(int count, uint32_t mabTime);
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
void uart5_tx(const uint8_t *b, int len);
void uart5_tx_break(int count, uint32_t mabTime);
#endif  // HAS_KINETISK_UART5

#if defined(HAS_KINETISK_LPUART0)
void lpuart0_tx(const uint8_t *b, int len);
void lpuart0_tx_break(int count, uint32_t mabTime);
#endif  // HAS_KINETISK_LPUART0

// Used by the RX ISRs.
Receiver *volatile rxInstances[6]{nullptr};

Receiver::Receiver(HardwareSerial &uart)
    : TeensyDMX(uart),
      began_(false),
      state_{RecvStates::kIdle},
      buf1_{0},
      buf2_{0},
      activeBuf_(buf1_),
      inactiveBuf_(buf2_),
      activeBufIndex_(0),
      packetSize_(0),
      packetTimestamp_(0),
      lastBreakStartTime_(0),
      breakStartTime_(0),
      lastSlotEndTime_(0),
      connected_(false),
      connectChangeFunc_{nullptr},
      packetTimeoutCount_(0),
      framingErrorCount_(0),
      shortPacketCount_(0),
      responderCount_(0),
      responderOutBufLen_{0},
      setTXNotRXFunc_(nullptr) {
  switch(serialIndex_) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
    case 0:
      txFunc_ = uart0_tx;
      txBreakFunc_ = uart0_tx_break;
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
    case 1:
      txFunc_ = uart1_tx;
      txBreakFunc_ = uart1_tx_break;
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
    case 2:
      txFunc_ = uart2_tx;
      txBreakFunc_ = uart2_tx_break;
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      txFunc_ = uart3_tx;
      txBreakFunc_ = uart3_tx_break;
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
    case 4:
      txFunc_ = uart4_tx;
      txBreakFunc_ = uart4_tx_break;
      break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
    case 5:
      txFunc_ = uart5_tx;
      txBreakFunc_ = uart5_tx_break;
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      txFunc_ = lpuart0_tx;
      txBreakFunc_ = lpuart0_tx_break;
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0

    default:
      txFunc_ = nullptr;
      txBreakFunc_ = nullptr;
  }
}

Receiver::~Receiver() {
  end();
}

// RX control states
#define UART_C2_RX_ENABLE UART_C2_RE | UART_C2_RIE | UART_C2_ILIE | UART_C2_TE
#define LPUART_CTRL_RX_ENABLE \
  LPUART_CTRL_RE | LPUART_CTRL_RIE | LPUART_CTRL_ILIE | LPUART_CTRL_TE

// Must define ACTIVATE_UART_RX_SERIAL_ERROR_N
#define ACTIVATE_UART_RX_SERIAL(N)                               \
  /* Enable receive-only */                                      \
  UART##N##_C2 = UART_C2_RX_ENABLE;                              \
  attachInterruptVector(IRQ_UART##N##_STATUS, uart##N##_rx_isr); \
  /* Enable interrupt on frame error */                          \
  UART##N##_C3 |= UART_C3_FEIE;                                  \
  ACTIVATE_UART_RX_SERIAL_ERROR_##N

#define ACTIVATE_UART_RX_SERIAL_ERROR(N)                           \
  attachInterruptVector(IRQ_UART##N##_ERROR, uart##N##_rx_isr);    \
  /* We fill bytes from the buffer in the framing error ISR, so we \
   * can set to the same priority. */                              \
  NVIC_SET_PRIORITY(IRQ_UART##N##_ERROR,                           \
                    NVIC_GET_PRIORITY(IRQ_UART##N##_STATUS));      \
  NVIC_ENABLE_IRQ(IRQ_UART##N##_ERROR);

#define ACTIVATE_LPUART_RX_SERIAL(N)                           \
  /* Enable receive-only and interrupt on frame error */       \
  LPUART##N##_CTRL = LPUART_CTRL_RX_ENABLE | LPUART_CTRL_FEIE; \
  attachInterruptVector(IRQ_LPUART##N, lpuart##N##_rx_isr);

void Receiver::begin() {
  if (began_) {
    return;
  }
  began_ = true;

  if (serialIndex_ < 0) {
    return;
  }

  resetPacketCount();

  // Set up the instance for the ISRs
  Receiver *r = rxInstances[serialIndex_];
  rxInstances[serialIndex_] = this;
  if (r != nullptr && r != this) {  // NOTE: Shouldn't be able to be 'this'
    r->end();
  }

  state_ = RecvStates::kIdle;
  activeBufIndex_ = 0;
  uart_.begin(kSlotsBaud, kSlotsFormat);

  // Reset "previous" state
  // NOTE: Any tampering with UART_C2 must be done after the serial port
  //       is activated because setting ILIE to 0 seems to lock things up
  setConnected(false);

  switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0)
    case 0:
#define ACTIVATE_UART_RX_SERIAL_ERROR_0 ACTIVATE_UART_RX_SERIAL_ERROR(0)
      ACTIVATE_UART_RX_SERIAL(0)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_0
      break;
#elif defined(HAS_KINETISL_UART0)
    case 0:
#define ACTIVATE_UART_RX_SERIAL_ERROR_0
      ACTIVATE_UART_RX_SERIAL(0)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_0
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1)
    case 1:
#define ACTIVATE_UART_RX_SERIAL_ERROR_1 ACTIVATE_UART_RX_SERIAL_ERROR(1)
      ACTIVATE_UART_RX_SERIAL(1)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_1
      break;
#elif defined(HAS_KINETISL_UART1)
    case 1:
#define ACTIVATE_UART_RX_SERIAL_ERROR_1
      ACTIVATE_UART_RX_SERIAL(1)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_1
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2)
    case 2:
#define ACTIVATE_UART_RX_SERIAL_ERROR_2 ACTIVATE_UART_RX_SERIAL_ERROR(2)
      ACTIVATE_UART_RX_SERIAL(2)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_2
      break;
#elif defined(HAS_KINETISL_UART2)
    case 2:
#define ACTIVATE_UART_RX_SERIAL_ERROR_2
      ACTIVATE_UART_RX_SERIAL(2)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_2
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
#define ACTIVATE_UART_RX_SERIAL_ERROR_3 ACTIVATE_UART_RX_SERIAL_ERROR(3)
      ACTIVATE_UART_RX_SERIAL(3)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_3
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
    case 4:
#define ACTIVATE_UART_RX_SERIAL_ERROR_4 ACTIVATE_UART_RX_SERIAL_ERROR(4)
      ACTIVATE_UART_RX_SERIAL(4)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_4
      break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
    case 5:
#define ACTIVATE_UART_RX_SERIAL_ERROR_5 ACTIVATE_UART_RX_SERIAL_ERROR(5)
      ACTIVATE_UART_RX_SERIAL(5)
#undef ACTIVATE_UART_RX_SERIAL_ERROR_5
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      ACTIVATE_LPUART_RX_SERIAL(0)
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }

  // Enable receive
  setTXNotRX(false);
}

// Undefine these macros
#undef ACTIVATE_UART_RX_SERIAL
#undef ACTIVATE_UART_RX_SERIAL_ERROR
#undef ACTIVATE_LPUART_RX_SERIAL
#undef UART_C2_RX_ENABLE
#undef LPUART_CTRL_RX_ENABLE

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

  uart_.end();

  switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0)
    case 0:
      // Disable UART0 interrupt on frame error
      UART0_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
      break;
#elif defined(HAS_KINETISL_UART0)
    case 0:
      // Disable UART0 interrupt on frame error
      UART0_C3 &= ~UART_C3_FEIE;
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1)
    case 1:
      UART1_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
      break;
#elif defined(HAS_KINETISL_UART1)
    case 1:
      UART1_C3 &= ~UART_C3_FEIE;
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2)
    case 2:
      UART2_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
      break;
#elif defined(HAS_KINETISL_UART2)
    case 2:
      UART2_C3 &= ~UART_C3_FEIE;
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      UART3_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART3_ERROR);
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
    case 4:
      UART4_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART4_ERROR);
      break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
    case 5:
      UART5_C3 &= ~UART_C3_FEIE;
      NVIC_DISABLE_IRQ(IRQ_UART5_ERROR);
      break;
#endif  // HAS_KINETISK_UART5
// NOTE: Nothing needed for LPUART0
  }

  // Remove the reference from the instances,
  // but only if we're the ones who added it
  if (rxInstances[serialIndex_] == this) {
    rxInstances[serialIndex_] = nullptr;
  }

  setConnected(false);
}

int Receiver::readPacket(uint8_t *buf, int startChannel, int len) {
  if (packetSize_ <= 0) {
    return -1;
  }
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
        memcpy(buf, &inactiveBuf_[startChannel], len);
        retval = len;
      }
      packetSize_ = 0;
    }
  //}
  return retval;
}

uint8_t Receiver::get(int channel) const {
  if (lastPacketSize_ <= 0) {
    return 0;
  }
  if (channel < 0 || kMaxDMXPacketSize <= channel) {
    return 0;
  }

  uint8_t b = 0;
  Lock lock{*this};
  //{
    // Since channel >= 0, lastPacketSize_ > channel implies lastPacketSize_ > 0
    if (channel < lastPacketSize_) {
      b = inactiveBuf_[channel];
    }
  //}
  return b;
}

std::shared_ptr<Responder> Receiver::setResponder(
    uint8_t startCode, std::shared_ptr<Responder> r) {
  // For a null responder, delete any current one for this start code
  if (r == nullptr) {
    if (responders_ == nullptr) {
      return nullptr;
    }

    Lock lock{*this};

    // Replace any previous responder
    std::shared_ptr<Responder> old{responders_[startCode]};
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
    responders_.reset(new std::shared_ptr<Responder>[256]);
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
      return nullptr;
    }
    responderOutBufLen_ = outBufSize;
  }

  // If a responder is already set then the output buffer should be the
  // correct size
  std::shared_ptr<Responder> old{responders_[startCode]};
  responders_[startCode] = std::move(r);
  if (old == nullptr) {
    responderCount_++;
  }

  return old;
}

void Receiver::completePacket() {
  uint32_t t = millis();
  state_ = RecvStates::kIdle;

  // An empty packet isn't valid, there must be at least a start code
  if (activeBufIndex_ <= 0) {
    return;
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
  lastPacketSize_ = packetSize_ = activeBufIndex_;
  packetTimestamp_ = t;

  // Let the responder, if any, process the packet
  if (responders_ != nullptr) {
    Responder *r = responders_[inactiveBuf_[0]].get();
    if (r != nullptr) {
      r->receivePacket(inactiveBuf_, packetSize_);
      if (r->eatPacket()) {
        lastPacketSize_ = packetSize_ = 0;
      }
    }
  }

  activeBufIndex_ = 0;
}

void Receiver::checkPacketTimeout() {
  uint32_t t = micros();

  if (state_ == RecvStates::kBreak) {
    // This catches the case where a short BREAK is followed by a longer MAB
    if ((t - breakStartTime_) < 88 + 44) {
      framingErrorCount_++;
      completePacket();
      setConnected(false);
    }
  } else if (state_ == RecvStates::kData) {
    if ((t - breakStartTime_) > kMaxDMXPacketTime ||
        (t - lastSlotEndTime_) >= kMaxDMXIdleTime) {
      packetTimeoutCount_++;
      completePacket();
      setConnected(false);
    }
  }
}

void Receiver::receivePotentialBreak() {
  // A potential BREAK is detected when a stop bit is expected but not
  // received, and this happens after the start bit, nine bits, and the
  // missing stop bit, about 44us.
  // Note that breakStartTime_ only represents a potential BREAK start
  // time until we receive the first character.
  breakStartTime_ = feStartTime_ - 44;

  state_ = RecvStates::kBreak;

  // At this point, we don't know whether to keep or discard any collected
  // data because the BREAK may be invalid. In other words, don't make any
  // framing error or short packet decisions until we know the nature of
  // this BREAK.
}

void Receiver::receiveBadBreak() {
  // Not a break
  framingErrorCount_++;

  // Don't keep the packet
  // See: [BREAK timing at the receiver](http://www.rdmprotocol.org/forums/showthread.php?t=1292)
  activeBufIndex_ = 0;
  completePacket();

  // Consider this case as not seeing a break
  // This may be line noise, so now we can't tell for sure where the
  // last break was
  setConnected(false);
}

void Receiver::receiveByte(uint8_t b) {
  uint32_t eopTime = micros();

  // Bad breaks are detected when BREAK + MAB + character time is too short
  // BREAK: 88us
  // MAB: 8us
  // Character time: 44us

  switch (state_) {
    case RecvStates::kBreak:
      // This is only a rudimentary check for short BREAKs. It does not
      // detect short BREAKs followed by long MABs. It only detects
      // whether BREAK + MAB time is at least 88us + 8us.
      if ((eopTime - breakStartTime_) < 88 + 8 + 44) {
        // First byte is too early, discard any data
        receiveBadBreak();
        return;
      } else if (connected_) {  // This condition indicates we haven't
                                // seen some timeout
        // Complete any un-flushed bytes
        uint32_t dt = breakStartTime_ - lastBreakStartTime_;
        if (dt < kMinDMXPacketTime) {
          shortPacketCount_++;
          // Discard the data
          activeBufIndex_ = 0;
        } else if (dt > kMaxDMXPacketTime) {
          // NOTE: Zero-length packets will also trigger a timeout
          packetTimeoutCount_++;
          // Keep the data
        }
        completePacket();
      } else {
        activeBufIndex_ = 0;
      }
      lastBreakStartTime_ = breakStartTime_;
      setConnected(true);
      state_ = RecvStates::kData;
      break;

    case RecvStates::kData:
      // Checking this here accounts for buffered input, where several
      // bytes come in at the same time
      if (static_cast<int>(eopTime - breakStartTime_) <
          88 + 8 + 44 + 44*activeBufIndex_) {
        // First byte is too early, discard any data
        receiveBadBreak();
        return;
      }
      // NOTE: Don't need to check for inter-slot MARK time being
      //       too large because the IDLE detection will catch that
      break;

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
    packetTimeoutCount_++;
    completePacket();
    return;
  }

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
    r = responders_[activeBuf_[0]].get();
  }
  if (r == nullptr) {
    if (packetFull) {
      completePacket();
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
      completePacket();
    }
    return;
  }
  completePacket();  // This is probably the best option, even though there may
                     // be more bytes
  if (txFunc_ == nullptr) {
    return;
  }

  // Do the response
  if (r->isSendBreakForLastPacket()) {
    if (txBreakFunc_ == nullptr) {
      return;
    }

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
    txBreakFunc_(r->breakLength(), r->markAfterBreakTime());
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
  txFunc_(responderOutBuf_.get(), respLen);
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

void Receiver::disableIRQs() const {
  if (!began_) {
    return;
  }
  switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0)
    case 0:
      NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART0_ERROR);
      break;
#elif defined(HAS_KINETISL_UART0)
    case 0:
      NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1)
    case 1:
      NVIC_DISABLE_IRQ(IRQ_UART1_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART1_ERROR);
      break;
#elif defined(HAS_KINETISL_UART1)
    case 1:
      NVIC_DISABLE_IRQ(IRQ_UART1_STATUS);
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2)
    case 2:
      NVIC_DISABLE_IRQ(IRQ_UART2_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART2_ERROR);
      break;
#elif defined(HAS_KINETISL_UART2)
    case 2:
      NVIC_DISABLE_IRQ(IRQ_UART2_STATUS);
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      NVIC_DISABLE_IRQ(IRQ_UART3_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART3_ERROR);
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
    case 4:
      NVIC_DISABLE_IRQ(IRQ_UART4_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART4_ERROR);
      break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
    case 5:
      NVIC_DISABLE_IRQ(IRQ_UART5_STATUS);
      NVIC_DISABLE_IRQ(IRQ_UART5_ERROR);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      NVIC_DISABLE_IRQ(IRQ_LPUART0);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }
}

void Receiver::enableIRQs() const {
  if (!began_) {
    return;
  }
  switch (serialIndex_) {
#if defined(HAS_KINETISK_UART0)
    case 0:
      NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART0_ERROR);
      break;
#elif defined(HAS_KINETISL_UART0)
    case 0:
      NVIC_ENABLE_IRQ(IRQ_UART0_STATUS);
      break;
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1)
    case 1:
      NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART1_ERROR);
      break;
#elif defined(HAS_KINETISL_UART1)
    case 1:
      NVIC_ENABLE_IRQ(IRQ_UART1_STATUS);
      break;
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2)
    case 2:
      NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART2_ERROR);
      break;
#elif defined(HAS_KINETISL_UART2)
    case 2:
      NVIC_ENABLE_IRQ(IRQ_UART2_STATUS);
      break;
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
    case 3:
      NVIC_ENABLE_IRQ(IRQ_UART3_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART3_ERROR);
      break;
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
    case 4:
      NVIC_ENABLE_IRQ(IRQ_UART4_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART4_ERROR);
      break;
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
    case 5:
      NVIC_ENABLE_IRQ(IRQ_UART5_STATUS);
      NVIC_ENABLE_IRQ(IRQ_UART5_ERROR);
      break;
#elif defined(HAS_KINETISK_LPUART0)
    case 5:
      NVIC_ENABLE_IRQ(IRQ_LPUART0);
      break;
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  }
}

// ---------------------------------------------------------------------------
//  UART0 synchronous TX
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)

#if defined(HAS_KINETISK_UART0_FIFO)
#define UART_SYNC_TX_SEND_FIFO_0 UART_SYNC_TX_SEND_FIFO(0)
#define UART_TX_FLUSH_FIFO_0 UART_TX_FLUSH_FIFO(0)
#else
#define UART_SYNC_TX_SEND_FIFO_0
#define UART_TX_FLUSH_FIFO_0
#endif  // HAS_KINETISK_UART0_FIFO

void uart0_tx(const uint8_t *b, int len) {
  UART_SYNC_TX(0, UART0_S1, UART_S1, UART0_D)
}

void uart0_tx_break(int count, uint32_t mabTime) {
  UART_TX_BREAK(0)
}

#undef UART_SYNC_TX_SEND_FIFO_0
#undef UART_TX_FLUSH_FIFO_0

#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

// ---------------------------------------------------------------------------
//  UART1 synchronous TX
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)

#if defined(HAS_KINETISK_UART1_FIFO)
#define UART_SYNC_TX_SEND_FIFO_1 UART_SYNC_TX_SEND_FIFO(1)
#define UART_TX_FLUSH_FIFO_1 UART_TX_FLUSH_FIFO(1)
#else
#define UART_SYNC_TX_SEND_FIFO_1
#define UART_TX_FLUSH_FIFO_1
#endif  // HAS_KINETISK_UART1_FIFO

void uart1_tx(const uint8_t *b, int len) {
  UART_SYNC_TX(1, UART1_S1, UART_S1, UART1_D)
}

void uart1_tx_break(int count, uint32_t mabTime) {
  UART_TX_BREAK(1)
}

#undef UART_SYNC_TX_SEND_FIFO_1
#undef UART_TX_FLUSH_FIFO_1

#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

// ---------------------------------------------------------------------------
//  UART2 synchronous TX
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)

#if defined(HAS_KINETISK_UART2_FIFO)
#define UART_SYNC_TX_SEND_FIFO_2 UART_SYNC_TX_SEND_FIFO(2)
#define UART_TX_FLUSH_FIFO_2 UART_TX_FLUSH_FIFO(2)
#else
#define UART_SYNC_TX_SEND_FIFO_2
#define UART_TX_FLUSH_FIFO_2
#endif  // HAS_KINETISK_UART2_FIFO

void uart2_tx(const uint8_t *b, int len) {
  UART_SYNC_TX(2, UART2_S1, UART_S1, UART2_D)
}

void uart2_tx_break(int count, uint32_t mabTime) {
  UART_TX_BREAK(2)
}

#undef UART_SYNC_TX_SEND_FIFO_2
#undef UART_TX_FLUSH_FIFO_2

#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

// ---------------------------------------------------------------------------
//  UART3 synchronous TX
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART3)

#define UART_SYNC_TX_SEND_FIFO_3

void uart3_tx(const uint8_t *b, int len) {
  UART_SYNC_TX(3, UART3_S1, UART_S1, UART3_D)
}

#undef UART_SYNC_TX_SEND_FIFO_3

#define UART_TX_FLUSH_FIFO_3

void uart3_tx_break(int count, uint32_t mabTime) {
  UART_TX_BREAK(3)
}

#undef UART_TX_FLUSH_FIFO_3

#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 synchronous TX
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART4)

#define UART_SYNC_TX_SEND_FIFO_4

void uart4_tx(const uint8_t *b, int len) {
  UART_SYNC_TX(4, UART4_S1, UART_S1, UART4_D)
}

#undef UART_SYNC_TX_SEND_FIFO_4

#define UART_TX_FLUSH_FIFO_4

void uart4_tx_break(int count, uint32_t mabTime) {
  UART_TX_BREAK(4)
}

#undef UART_TX_FLUSH_FIFO_4

#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 synchronous TX
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART5)

#define UART_SYNC_TX_SEND_FIFO_5

void uart5_tx(const uint8_t *b, int len) {
  UART_SYNC_TX(5, UART5_S1, UART_S1, UART5_D)
}

#undef UART_SYNC_TX_SEND_FIFO_5

#define UART_TX_FLUSH_FIFO_5

void uart5_tx_break(int count, uint32_t mabTime) {
  UART_TX_BREAK(5)
}

#undef UART_TX_FLUSH_FIFO_5

#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 synchronous TX
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_LPUART0)

#define UART_SYNC_TX_SEND_FIFO_0

void lpuart0_tx(const uint8_t *b, int len) {
  UART_SYNC_TX(0, LPUART0_STAT, LPUART_STAT, LPUART0_DATA)
}

#undef UART_SYNC_TX_SEND_FIFO_0

void lpuart0_tx_break(int count, uint32_t mabTime) {
  LPUART_TX_BREAK(0)
}

#endif  // HAS_KINETISK_LPUART0

// ---------------------------------------------------------------------------
//  UART0 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)

#if defined(HAS_KINETISL_UART0)
#define UART_RX_CLEAR_ERRORS_0 UART0_S1 |= (UART_S1_FE | UART_S1_IDLE);
#else
// Reading a byte clears interrupt flags
#define UART_RX_CLEAR_ERRORS_0
#endif  // HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART0_FIFO)
#define UART_RX_ERROR_FLUSH_FIFO_0 UART_RX_ERROR_FLUSH_FIFO(0)
#define UART_RX_0 UART_RX_WITH_FIFO(0)
#else
#define UART_RX_ERROR_FLUSH_FIFO_0
#define UART_RX_0 UART_RX_NO_FIFO(0, UART_S1, UART0_D)
#if defined(HAS_KINETISL_UART0)
#define UART_RX_CLEAR_IDLE_0 UART0_S1 |= UART_S1_IDLE;
#else
#define UART_RX_CLEAR_IDLE_0 UART0_D;
#endif  // HAS_KINETISL_UART0
#endif  // HAS_KINETISK_UART0_FIFO

#if defined(__MK20DX128__) || defined(__MK20DX256__)
#define UART_RX_TEST_R8_0 ((UART0_C3 & UART_C3_R8) != 0)
#else
#define UART_RX_TEST_R8_0 (true)
#endif

void uart0_rx_isr() {
  uint8_t status = UART0_S1;
  UART_RX(0, 0, UART_S1, UART0_D)
}

#undef UART_RX_CLEAR_ERRORS_0
#undef UART_RX_ERROR_FLUSH_FIFO_0
#undef UART_RX_0
#undef UART_RX_CLEAR_IDLE_0
#undef UART_RX_TEST_R8_0

#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

// ---------------------------------------------------------------------------
//  UART1 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)

// Reading a byte clears interrupt flags
#define UART_RX_CLEAR_ERRORS_1
#if defined(HAS_KINETISK_UART1_FIFO)
#define UART_RX_ERROR_FLUSH_FIFO_1 UART_RX_ERROR_FLUSH_FIFO(1)
#define UART_RX_1 UART_RX_WITH_FIFO(1)
#else
#define UART_RX_ERROR_FLUSH_FIFO_1
#define UART_RX_1 UART_RX_NO_FIFO(1, UART_S1, UART1_D)
#define UART_RX_CLEAR_IDLE_1 UART1_D;
#endif  // HAS_KINETISK_UART1_FIFO

#if defined(__MK20DX128__) || defined(__MK20DX256__)
#define UART_RX_TEST_R8_1 ((UART1_C3 & UART_C3_R8) != 0)
#else
#define UART_RX_TEST_R8_1 (true)
#endif

void uart1_rx_isr() {
  uint8_t status = UART1_S1;
  UART_RX(1, 1, UART_S1, UART1_D)
}

#undef UART_RX_CLEAR_ERRORS_1
#undef UART_RX_ERROR_FLUSH_FIFO_1
#undef UART_RX_1
#undef UART_RX_CLEAR_IDLE_1
#undef UART_RX_TEST_R8_1

#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

// ---------------------------------------------------------------------------
//  UART2 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)

// Reading a byte clears interrupt flags
#define UART_RX_CLEAR_ERRORS_2
#if defined(HAS_KINETISK_UART2_FIFO)
#define UART_RX_ERROR_FLUSH_FIFO_2 UART_RX_ERROR_FLUSH_FIFO(2)
#define UART_RX_2 UART_RX_WITH_FIFO(2)
#else
#define UART_RX_ERROR_FLUSH_FIFO_2
#define UART_RX_2 UART_RX_NO_FIFO(2, UART_S1, UART2_D)
#define UART_RX_CLEAR_IDLE_2 UART2_D;
#endif  // HAS_KINETISK_UART2_FIFO

#if defined(__MK20DX128__) || defined(__MK20DX256__)
#define UART_RX_TEST_R8_2 ((UART2_C3 & UART_C3_R8) != 0)
#else
#define UART_RX_TEST_R8_2 (true)
#endif

void uart2_rx_isr() {
  uint8_t status = UART2_S1;
  UART_RX(2, 2, UART_S1, UART2_D)
}

#undef UART_RX_CLEAR_ERRORS_2
#undef UART_RX_ERROR_FLUSH_FIFO_2
#undef UART_RX_2
#undef UART_RX_CLEAR_IDLE_2
#undef UART_RX_TEST_R8_2

#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

// ---------------------------------------------------------------------------
//  UART3 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART3)

// Reading a byte clears interrupt flags
#define UART_RX_CLEAR_ERRORS_3
#define UART_RX_ERROR_FLUSH_FIFO_3
#define UART_RX_3 UART_RX_NO_FIFO(3, UART_S1, UART3_D)
#define UART_RX_CLEAR_IDLE_3 UART3_D;
#define UART_RX_TEST_R8_3 (true)

void uart3_rx_isr() {
  uint8_t status = UART3_S1;
  UART_RX(3, 3, UART_S1, UART3_D)
}

#undef UART_RX_CLEAR_ERRORS_3
#undef UART_RX_ERROR_FLUSH_FIFO_3
#undef UART_RX_3
#undef UART_RX_CLEAR_IDLE_3
#undef UART_RX_TEST_R8_3

#endif  // HAS_KINETISK_UART3

// ---------------------------------------------------------------------------
//  UART4 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART4)

// Reading a byte clears interrupt flags
#define UART_RX_CLEAR_ERRORS_4
#define UART_RX_ERROR_FLUSH_FIFO_4
#define UART_RX_4 UART_RX_NO_FIFO(4, UART_S1, UART4_D)
#define UART_RX_CLEAR_IDLE_4 UART4_D;
#define UART_RX_TEST_R8_4 (true)

void uart4_rx_isr() {
  uint8_t status = UART4_S1;
  UART_RX(4, 4, UART_S1, UART4_D)
}

#undef UART_RX_CLEAR_ERRORS_4
#undef UART_RX_ERROR_FLUSH_FIFO_4
#undef UART_RX_4
#undef UART_RX_CLEAR_IDLE_4
#undef UART_RX_TEST_R8_4

#endif  // HAS_KINETISK_UART4

// ---------------------------------------------------------------------------
//  UART5 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_UART5)

// Reading a byte clears interrupt flags
#define UART_RX_CLEAR_ERRORS_5
#define UART_RX_ERROR_FLUSH_FIFO_5
#define UART_RX_5 UART_RX_NO_FIFO(5, UART_S1, UART5_D)
#define UART_RX_CLEAR_IDLE_5 UART5_D;
#define UART_RX_TEST_R8_5 (true)

void uart5_rx_isr() {
  uint8_t status = UART5_S1;
  UART_RX(5, 5, UART_S1, UART5_D)
}

#undef UART_RX_CLEAR_ERRORS_5
#undef UART_RX_ERROR_FLUSH_FIFO_5
#undef UART_RX_5
#undef UART_RX_CLEAR_IDLE_5
#undef UART_RX_TEST_R8_5

#endif  // HAS_KINETISK_UART5

// ---------------------------------------------------------------------------
//  LPUART0 RX ISR
// ---------------------------------------------------------------------------

#if defined(HAS_KINETISK_LPUART0)

#define UART_RX_CLEAR_ERRORS_0 \
  LPUART0_STAT |= (LPUART_STAT_FE | LPUART_STAT_IDLE);
#define UART_RX_ERROR_FLUSH_FIFO_0
#define UART_RX_0 UART_RX_NO_FIFO(0, LPUART_STAT, LPUART0_DATA)
#define UART_RX_CLEAR_IDLE_0 LPUART0_STAT |= LPUART_STAT_IDLE;
#define UART_RX_TEST_R8_0 (true)

void lpuart0_rx_isr() {
  uint32_t status = LPUART0_STAT;
  UART_RX(5, 0, LPUART_STAT, LPUART0_DATA)
}

#undef UART_RX_CLEAR_ERRORS_0
#undef UART_RX_ERROR_FLUSH_FIFO_0
#undef UART_RX_0
#undef UART_RX_CLEAR_IDLE_0
#undef UART_RX_TEST_R8_0

#endif  // HAS_KINETISK_LPUART0

}  // namespace teensydmx
}  // namespace qindesign
