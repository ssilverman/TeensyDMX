// This file is part of the TeensyDMX library.
// (c) 2019 Shawn Silverman

#if defined(__IMXRT1062__) || defined(__IMXRT1052__) || defined(__MK66FX1M0__)

#include "LPUARTSendHandler.h"

// C++ includes
#include <cstdint>

#include <Arduino.h>
#include "TeensyDMX.h"

namespace qindesign {
namespace teensydmx {

// TX control states
#define LPUART_CTRL_TX_ENABLE     (LPUART_CTRL_TE)
#define LPUART_CTRL_TX_ACTIVE     ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TIE))
#define LPUART_CTRL_TX_COMPLETING ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TCIE))
#define LPUART_CTRL_TX_INACTIVE   (LPUART_CTRL_TX_ENABLE)

extern const uint32_t kDefaultBreakBaud;
extern const uint32_t kDefaultBreakFormat;
extern const uint32_t kSlotsBaud;
extern const uint32_t kSlotsFormat;

void LPUARTSendHandler::start() {
  if (!serialParamsSet_) {
    sender_->uart_.begin(kDefaultBreakBaud, kDefaultBreakFormat);
    breakSerialParams_.getFrom(port_);
    sender_->uart_.begin(kSlotsBaud, kSlotsFormat);
    slotsSerialParams_.getFrom(port_);
    serialParamsSet_ = true;
  } else {
    sender_->uart_.begin(kSlotsBaud, kSlotsFormat);
  }

#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
  // Calculate the FIFO size now that the peripheral has been enabled and we can
  // access the registers
  if (!fifoSizeSet_) {
    // Calculate the TX FIFO size based on the TXFIFOSIZE bits
    int bits = (port_->FIFO >> 4) & 0x07;  // TXFIFOSIZE
    if (bits == 0) {
      fifoSize_ = 1;
    } else {
      fifoSize_ = 1 << (bits + 1);
    }

    fifoSizeSet_ = true;
  }
#endif  // __IMXRT1062__ || __IMXRT1052__

  attachInterruptVector(irq_, irqHandler_);
}

void LPUARTSendHandler::end() const {
  sender_->uart_.end();
}

void LPUARTSendHandler::setActive() const {
  port_->CTRL = LPUART_CTRL_TX_ACTIVE;
}

void LPUARTSendHandler::setIRQsEnabled(bool flag) const {
  if (flag) {
    NVIC_ENABLE_IRQ(irq_);
  } else {
    NVIC_DISABLE_IRQ(irq_);
  }
}

int LPUARTSendHandler::priority() const {
  return NVIC_GET_PRIORITY(irq_);
}

void LPUARTSendHandler::irqHandler() {
  uint32_t status = port_->STAT;
  uint32_t control = port_->CTRL;

  // If the transmit buffer is empty
  if ((control & LPUART_CTRL_TIE) != 0 && (status & LPUART_STAT_TDRE) != 0) {
    switch (sender_->state_) {
      case Sender::XmitStates::kBreak:
        if (sender_->intervalTimer_.begin(
                [&]() {
                  if (sender_->state_ == Sender::XmitStates::kBreak) {
                    port_->CTRL &= ~LPUART_CTRL_TXINV;
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
                  port_->CTRL = LPUART_CTRL_TX_ACTIVE;
                },
                sender_->breakTime_)) {
          port_->CTRL = LPUART_CTRL_TX_INACTIVE;
          // Invert the line as close as possible to the
          // interrupt start
          port_->CTRL |= LPUART_CTRL_TXINV;
          sender_->breakStartTime_ = micros();
        } else {
          // Starting the timer failed, revert to the original way
          breakSerialParams_.apply(port_);
          port_->DATA = 0;
          port_->CTRL = LPUART_CTRL_TX_COMPLETING;
          sender_->breakStartTime_ = micros();
        }
        break;

      case Sender::XmitStates::kMAB:  // Shouldn't be needed
        sender_->state_ = Sender::XmitStates::kData;
        port_->CTRL = LPUART_CTRL_TX_ACTIVE;
        break;

      case Sender::XmitStates::kData:
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
        do {
          if (sender_->outputBufIndex_ >= sender_->packetSize_) {
            port_->CTRL = LPUART_CTRL_TX_COMPLETING;
            break;
          }
          port_->DATA = sender_->outputBuf_[sender_->outputBufIndex_++];
        } while (((port_->WATER >> 8) & 0x07) < fifoSize_);  // TXCOUNT
#else  // No FIFO
        if (sender_->outputBufIndex_ >= sender_->packetSize_) {
          port_->CTRL = LPUART_CTRL_TX_COMPLETING;
        } else {
          port_->DATA = sender_->outputBuf_[sender_->outputBufIndex_++];
          if (sender_->outputBufIndex_ >= sender_->packetSize_) {
            port_->CTRL = LPUART_CTRL_TX_COMPLETING;
          }
        }
#endif
        break;

      case Sender::XmitStates::kIdle: {
        // Pause management
        if (sender_->paused_) {
          port_->CTRL = LPUART_CTRL_TX_INACTIVE;
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
          port_->CTRL = LPUART_CTRL_TX_INACTIVE;
          if (sender_->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite BREAK time
            if (!sender_->intervalTimer_.begin(
                    [&]() {
                      sender_->intervalTimer_.end();
                      port_->CTRL = LPUART_CTRL_TX_ACTIVE;
                    },
                    sender_->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              port_->CTRL = LPUART_CTRL_TX_ACTIVE;
            }
          }
        } else {
          // No delay necessary
          port_->CTRL = LPUART_CTRL_TX_ACTIVE;
        }
        break;
      }

      default:
        break;
    }
  }

  // If transmission is complete
  if ((control & LPUART_CTRL_TCIE) != 0 && (status & LPUART_STAT_TC) != 0) {
    switch (sender_->state_) {
      case Sender::XmitStates::kBreak:
        sender_->state_ = Sender::XmitStates::kData;
        slotsSerialParams_.apply(port_);
        break;

      case Sender::XmitStates::kMAB:  // Shouldn't be needed
        sender_->state_ = Sender::XmitStates::kData;
        slotsSerialParams_.apply(port_);
        break;

      case Sender::XmitStates::kData:
        sender_->completePacket();
        break;

      case Sender::XmitStates::kIdle:
        break;

      default:
        break;
    }
    port_->CTRL = LPUART_CTRL_TX_ACTIVE;
  }
}

#undef LPUART_CTRL_TX_ENABLE
#undef LPUART_CTRL_TX_ACTIVE
#undef LPUART_CTRL_TX_COMPLETING
#undef LPUART_CTRL_TX_INACTIVE

}  // namespace teensydmx
}  // namespace qindesign

#endif  // __IMXRT1062__ || __IMXRT1052__ || __MK66FX1M0__
