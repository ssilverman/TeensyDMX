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

constexpr uint32_t kBreakBaud   = 1000000 / 12;
constexpr uint32_t kBreakFormat = SERIAL_8N1;
constexpr uint32_t kSlotsBaud   = 250000;
constexpr uint32_t kSlotsFormat = SERIAL_8N2;

// TX control states
#define UART_C2_TX_ENABLE     (UART_C2_TE)
#define UART_C2_TX_ACTIVE     ((UART_C2_TX_ENABLE) | (UART_C2_TIE))
#define UART_C2_TX_COMPLETING ((UART_C2_TX_ENABLE) | (UART_C2_TCIE))
#define UART_C2_TX_INACTIVE   (UART_C2_TX_ENABLE)

// Used by the TX ISR's.
Sender *txInstances[3]{nullptr};

void Sender::begin() {
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

void Sender::end() {
  int index = serialIndex(uart_);
  if (index < 0) {
    return;
  }

  // Remove any chance that our TX ISR calls begin after end() is called
  switch (index) {
    case 0:
      NVIC_DISABLE_IRQ(IRQ_UART0_STATUS);
      break;
    case 1:
      NVIC_DISABLE_IRQ(IRQ_UART1_STATUS);
      break;
    case 2:
      NVIC_DISABLE_IRQ(IRQ_UART2_STATUS);
      break;
  }

  if (!began_) {
    return;
  }
  began_ = false;

  // Remove the reference from the instances
  txInstances[index] = nullptr;

  uart_.end();
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

  memcpy(const_cast<uint8_t*>(outputBuf_ + startChannel), values, len);
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

      case Sender::XmitStates::kIdle:
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART0_D = 0;
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

      case Sender::XmitStates::kIdle:
        break;
    }
  }
#endif  // HAS_KINETISK_UART0_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);
        break;

      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
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
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART1_D = 0;
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
        break;
    }
  }
#endif  // HAS_KINETISK_UART1_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);
        break;

      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
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
        break;
    }
  }
#else
  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kBreak:
        UART2_D = 0;
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
        break;
    }
  }
#endif  // HAS_KINETISK_UART2_FIFO

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (instance->state_) {
      case Sender::XmitStates::kIdle:
        instance->state_ = Sender::XmitStates::kBreak;
        instance->uart_.begin(kBreakBaud, kBreakFormat);
        break;

      case Sender::XmitStates::kBreak:
        instance->state_ = Sender::XmitStates::kData;
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);
        break;

      case Sender::XmitStates::kData:
        break;
    }
    UART2_C2 = UART_C2_TX_ACTIVE;
  }
}

}  // namespace teensydmx
}  // namespace qindesign
