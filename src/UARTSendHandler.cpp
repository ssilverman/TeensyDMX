// This file is part of the TeensyDMX library.
// (c) 2019 Shawn Silverman

#if defined(__MK20DX128__) || defined(__MK20DX256__) || \
    defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

#include "UARTSendHandler.h"

// C++ includes
#include <cstdint>

#include <Arduino.h>
#include "TeensyDMX.h"

namespace qindesign {
namespace teensydmx {

#define UART_C2_TX_ENABLE     (UART_C2_TE)
#define UART_C2_TX_ACTIVE     ((UART_C2_TX_ENABLE) | (UART_C2_TIE))
#define UART_C2_TX_COMPLETING ((UART_C2_TX_ENABLE) | (UART_C2_TCIE))
#define UART_C2_TX_INACTIVE   (UART_C2_TX_ENABLE)

extern const uint32_t kDefaultBreakBaud;
extern const uint32_t kDefaultBreakFormat;
extern const uint32_t kSlotsBaud;
extern const uint32_t kSlotsFormat;

void UARTSendHandler::start() {
  // Set the serial parameters for the two modes
  if (!serialParamsSet_) {
    sender_->uart_.begin(kDefaultBreakBaud, kDefaultBreakFormat);
    breakSerialParams_.getFrom(serialIndex_, port_);
    sender_->uart_.begin(kSlotsBaud, kSlotsFormat);
    slotsSerialParams_.getFrom(serialIndex_, port_);
    serialParamsSet_ = true;
  } else {
    sender_->uart_.begin(kSlotsBaud, kSlotsFormat);
  }

#if defined(KINETISK)
  // Calculate the FIFO size now that the peripheral has been enabled and we can
  // access the registers
  if (!fifoSizeSet_) {
    // Calculate the TX FIFO size based on the TXFIFOSIZE bits
    int bits = (port_->PFIFO >> 4) & 0x07;  // TXFIFOSIZE
    if (bits == 0 || bits == 7) {
      fifoSize_ = 1;
    } else {
      fifoSize_ = 1 << (bits + 1);
    }

    fifoSizeSet_ = true;
  }
#endif  // KINETISK

  attachInterruptVector(irq_, irqHandler_);
}

void UARTSendHandler::end() const {
  sender_->uart_.end();
}

void UARTSendHandler::setActive() const {
  port_->C2 = UART_C2_TX_ACTIVE;
}

void UARTSendHandler::setIRQsEnabled(bool flag) const {
  if (flag) {
    NVIC_ENABLE_IRQ(irq_);
  } else {
    NVIC_DISABLE_IRQ(irq_);
  }
}

int UARTSendHandler::priority() const {
  return NVIC_GET_PRIORITY(irq_);
}

void UARTSendHandler::irqHandler() {
  uint8_t status = port_->S1;
  uint8_t control = port_->C2;

  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (sender_->state_) {
      case Sender::XmitStates::kBreak:
        if (sender_->intervalTimer_.begin(
                [&]() {
                  if (sender_->state_ == Sender::XmitStates::kBreak) {
                    port_->C3 &= ~UART_C3_TXINV;
                    sender_->state_ = Sender::XmitStates::kMAB;
                    if (sender_->intervalTimer_.restart(
                            sender_->adjustedMABTime_)) {
                      return;
                    }
                    // We shouldn't delay as an alternative because
                    // that might mean we delay too long
                  }
                  sender_->intervalTimer_.end();
                  sender_->state_ = Sender::XmitStates::kData;
                  port_->C2 = UART_C2_TX_ACTIVE;
                },
                sender_->breakTime_)) {
          port_->C2 = UART_C2_TX_INACTIVE;
          // Invert the line as close as possible to the
          // interrupt start
          port_->C3 |= UART_C3_TXINV;
          sender_->breakStartTime_ = micros();
        } else {
          // Starting the timer failed, revert to the original way
          breakSerialParams_.apply(serialIndex_, port_);
          port_->D = 0;
          port_->C2 = UART_C2_TX_COMPLETING;
          sender_->breakStartTime_ = micros();
        }
        break;

      case Sender::XmitStates::kMAB:  // Shouldn't be needed
        sender_->state_ = Sender::XmitStates::kData;
        port_->C2 = UART_C2_TX_ACTIVE;
        break;

      case Sender::XmitStates::kData:
#if defined(KINETISK)
        if (fifoSize_ > 1) {
          do {
            if (sender_->outputBufIndex_ >= sender_->packetSize_) {
              port_->C2 = UART_C2_TX_COMPLETING;
              break;
            }
            port_->S1;
            port_->D = sender_->outputBuf_[sender_->outputBufIndex_++];
          } while (port_->TCFIFO < fifoSize_);  // Transmit Count
        } else {  // No FIFO
          if (sender_->outputBufIndex_ >= sender_->packetSize_) {
            port_->C2 = UART_C2_TX_COMPLETING;
          } else {
            port_->D = sender_->outputBuf_[sender_->outputBufIndex_++];
            if (sender_->outputBufIndex_ >= sender_->packetSize_) {
              port_->C2 = UART_C2_TX_COMPLETING;
            }
          }
        }
#else  // No FIFO
        if (sender_->outputBufIndex_ >= sender_->packetSize_) {
          port_->C2 = UART_C2_TX_COMPLETING;
        } else {
          port_->D = sender_->outputBuf_[sender_->outputBufIndex_++];
          if (sender_->outputBufIndex_ >= sender_->packetSize_) {
            port_->C2 = UART_C2_TX_COMPLETING;
          }
        }
#endif
        break;

      case Sender::XmitStates::kIdle: {
        // Pause management
        if (sender_->paused_) {
          port_->C2 = UART_C2_TX_INACTIVE;
          return;
        }
        if (sender_->resumeCounter_ > 0) {
          if (--sender_->resumeCounter_ == 0) {
            sender_->paused_ = true;
          }
        }

        sender_->transmitting_ = true;
        sender_->state_ = Sender::XmitStates::kBreak;

        // Delay so that we can achieve the specified refresh rate
        uint32_t timeSinceBreak = micros() - sender_->breakStartTime_;
        if (timeSinceBreak < sender_->breakToBreakTime_) {
          port_->C2 = UART_C2_TX_INACTIVE;
          if (sender_->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!sender_->intervalTimer_.begin(
                    [&]() {
                      sender_->intervalTimer_.end();
                      port_->C2 = UART_C2_TX_ACTIVE;
                    },
                    sender_->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              port_->C2 = UART_C2_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          port_->C2 = UART_C2_TX_ACTIVE;
        }
        break;
      }

      default:
        break;
    }
  }

  // If transmission is complete
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) {
    switch (sender_->state_) {
      case Sender::XmitStates::kBreak:
        sender_->state_ = Sender::XmitStates::kData;
        slotsSerialParams_.apply(serialIndex_, port_);
        break;

      case Sender::XmitStates::kMAB:  // Shouldn't be needed
        sender_->state_ = Sender::XmitStates::kData;
        slotsSerialParams_.apply(serialIndex_, port_);
        break;

      case Sender::XmitStates::kData:
        sender_->completePacket();
        break;

      case Sender::XmitStates::kIdle:
        break;

      default:
        break;
    }
    port_->C2 = UART_C2_TX_ACTIVE;
  }
}

#undef UART_C2_TX_ENABLE
#undef UART_C2_TX_ACTIVE
#undef UART_C2_TX_COMPLETING
#undef UART_C2_TX_INACTIVE

}  // namespace teensydmx
}  // namespace qindesign

#endif  // __MK20DX128__ || __MK20DX256__ || __MKL26Z64__ || __MK64FX512__ ||
        // __MK66FX1M0__
