// This file is part of the TeensyDMX library.
// (c) 2019-2023 Shawn Silverman

#if defined(__IMXRT1062__) || defined(__IMXRT1052__) || defined(__MK66FX1M0__)

#include "LPUARTSendHandler.h"

#include <core_pins.h>

namespace qindesign {
namespace teensydmx {

// TX control states
#define LPUART_CTRL_TX_ENABLE     (LPUART_CTRL_TE)
#define LPUART_CTRL_TX_ACTIVE     ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TIE))
#define LPUART_CTRL_TX_COMPLETING ((LPUART_CTRL_TX_ENABLE) | (LPUART_CTRL_TCIE))
#define LPUART_CTRL_TX_INACTIVE   (LPUART_CTRL_TX_ENABLE)

extern const uint32_t kSlotsBaud;
extern const uint32_t kSlotsFormat;

// Disables all RX options for the given port. This is used before storing
// BREAK and slots serial port parameters.
static void disableRX(PortType *port) {
  port->CTRL &= ~(LPUART_CTRL_ORIE | LPUART_CTRL_NEIE | LPUART_CTRL_FEIE |
                  LPUART_CTRL_PEIE | LPUART_CTRL_RIE  | LPUART_CTRL_ILIE |
                  LPUART_CTRL_RE);
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
  port->CTRL &= ~(LPUART_CTRL_MA1IE | LPUART_CTRL_MA2IE);
#endif  // __IMXRT1062__ || __IMXRT1052__
}

void LPUARTSendHandler::start() {
  if (breakSerialParamsChanged_) {
    sender_->uart_.begin(sender_->breakBaud_, sender_->breakFormat_);
    disableRX(port_);
    breakSerialParams_.getFrom(port_);
    breakSerialParamsChanged_ = false;
  }
  if (!slotsSerialParamsSet_) {
    sender_->uart_.begin(kSlotsBaud, kSlotsFormat);
    disableRX(port_);
    slotsSerialParams_.getFrom(port_);
    slotsSerialParamsSet_ = true;
  } else {
    sender_->uart_.begin(kSlotsBaud, kSlotsFormat);
    disableRX(port_);
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
  port_->CTRL =
      (port_->CTRL | (LPUART_CTRL_TE | LPUART_CTRL_TIE)) & ~LPUART_CTRL_TCIE;
}

void LPUARTSendHandler::setInactive() const {
  port_->CTRL =
      (port_->CTRL | LPUART_CTRL_TE) & ~(LPUART_CTRL_TIE | LPUART_CTRL_TCIE);
}

void LPUARTSendHandler::setCompleting() const {
  port_->CTRL =
      (port_->CTRL | (LPUART_CTRL_TE | LPUART_CTRL_TCIE)) & ~LPUART_CTRL_TIE;
}

void LPUARTSendHandler::setIRQState(bool flag) const {
  if (flag) {
    NVIC_ENABLE_IRQ(irq_);
  } else {
    NVIC_DISABLE_IRQ(irq_);
  }
}

int LPUARTSendHandler::priority() const {
  return NVIC_GET_PRIORITY(irq_);
}

void LPUARTSendHandler::breakTimerCallback() const {
  if (sender_->state_ == Sender::XmitStates::kBreak) {
    port_->CTRL &= ~LPUART_CTRL_TXINV;
    sender_->state_ = Sender::XmitStates::kMAB;
    if (sender_->intervalTimer_.restart(sender_->adjustedMABTime_)) {
      return;
    }
    // The restart shouldn't fail because the timer is already
    // active, but we need to check it because the return value
    // is part of the API
    //
    // We shouldn't delay as an alternative because that might
    // mean we delay too long, however the MAB is most likely to
    // be too short in this case
  }
  sender_->intervalTimer_.end();
  sender_->state_ = Sender::XmitStates::kData;
  setActive();
}

void LPUARTSendHandler::breakTimerPreCallback() const {
  // Invert the line as close as possible to the timer start
  port_->CTRL |= LPUART_CTRL_TXINV;
  setInactive();
  sender_->breakStartTime_ = micros();
}

void LPUARTSendHandler::interSlotTimerCallback() const {
  sender_->intervalTimer_.end();
  sender_->state_ = Sender::XmitStates::kData;
  setActive();
}

void LPUARTSendHandler::rateTimerCallback() const {
  sender_->intervalTimer_.end();
  setActive();
}

void LPUARTSendHandler::irqHandler() const {
  uint32_t status = port_->STAT;
  uint32_t control = port_->CTRL;

  // If the transmit buffer is empty
  if ((control & LPUART_CTRL_TIE) != 0 && (status & LPUART_STAT_TDRE) != 0) {
    switch (sender_->state_) {
      case Sender::XmitStates::kBreak:
#ifndef TEENSYDMX_USE_PERIODICTIMER
        if (sender_->breakUseTimer_ &&
            sender_->intervalTimer_.begin(
                [this]() { breakTimerCallback(); },
                sender_->adjustedBreakTime_)) {
          breakTimerPreCallback();
#else
        if (sender_->breakUseTimer_ &&
            sender_->intervalTimer_.begin(
                [this]() { breakTimerCallback(); },
                sender_->breakTime_,
                [this]() { breakTimerPreCallback(); })) {
#endif  // !TEENSYDMX_USE_PERIODICTIMER
        } else {
          // Not using a timer or starting it failed;
          // revert to the original way
          breakSerialParams_.apply(port_);
          port_->DATA = 0;
          setCompleting();
          sender_->breakStartTime_ = micros();
        }
        break;

      case Sender::XmitStates::kMAB:  // Shouldn't be needed
        sender_->state_ = Sender::XmitStates::kData;
        setActive();
        break;

      case Sender::XmitStates::kData:
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
        if (sender_->interSlotTime_ == 0) {
          do {
            if (sender_->inactiveBufIndex_ >= sender_->inactivePacketSize_) {
              setCompleting();
              break;
            }
            port_->DATA = sender_->inactiveBuf_[sender_->inactiveBufIndex_++];
          } while (((port_->WATER >> 8) & 0x07) < fifoSize_);  // TXCOUNT
        } else {
          // Don't use the FIFO
          if (sender_->inactiveBufIndex_ < sender_->inactivePacketSize_) {
            port_->DATA = sender_->inactiveBuf_[sender_->inactiveBufIndex_++];
            if (sender_->inactiveBufIndex_ < sender_->inactivePacketSize_) {
              sender_->state_ = Sender::XmitStates::kInterSlot;
            }
          }
          setCompleting();
        }
#else  // No FIFO
        if (sender_->inactiveBufIndex_ < sender_->inactivePacketSize_) {
          port_->DATA = sender_->inactiveBuf_[sender_->inactiveBufIndex_++];
          if (sender_->inactiveBufIndex_ >= sender_->inactivePacketSize_) {
            setCompleting();
          } else if (sender_->interSlotTime_ != 0) {
            sender_->state_ = Sender::XmitStates::kInterSlot;
            setCompleting();
          }
        } else {
          setCompleting();
        }
#endif  // __IMXRT1062__ || __IMXRT1052__
        break;

      case Sender::XmitStates::kIdle: {
        // Pause management
        if (sender_->paused_) {
          setInactive();
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
        // including the MBB
        uint32_t timeSinceBreak = micros() - sender_->breakStartTime_;
        if (sender_->breakToBreakTime_ == UINT32_MAX) {
          // Infinite BREAK to BREAK time
          setInactive();
          return;
        }
        uint32_t delay = sender_->adjustedMBBTime_;
        if (timeSinceBreak + delay < sender_->breakToBreakTime_) {
          delay = sender_->breakToBreakTime_ - timeSinceBreak;
        }
        if (delay > 0) {
          setInactive();
          if (sender_->intervalTimer_.begin(
                  [this]() { rateTimerCallback(); },
                  delay)) {
            return;
          }
        }
        // Starting the timer failed or no delay is necessary
        setActive();
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

      case Sender::XmitStates::kInterSlot:
        setInactive();
        if (sender_->intervalTimer_.begin(
                [this]() { interSlotTimerCallback(); },
                sender_->adjustedInterSlotTime_)) {
          return;
        }
        sender_->state_ = Sender::XmitStates::kData;
        break;

      case Sender::XmitStates::kIdle:
        break;

      default:
        break;
    }
    setActive();
  }
}

#undef LPUART_CTRL_TX_ENABLE
#undef LPUART_CTRL_TX_ACTIVE
#undef LPUART_CTRL_TX_COMPLETING
#undef LPUART_CTRL_TX_INACTIVE

}  // namespace teensydmx
}  // namespace qindesign

#endif  // __IMXRT1062__ || __IMXRT1052__ || __MK66FX1M0__
