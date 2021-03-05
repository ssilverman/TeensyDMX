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

extern const uint32_t kSlotsBaud;
extern const uint32_t kSlotsFormat;

void UARTSendHandler::start() {
  // Set the serial parameters for the two modes
  if (breakSerialParamsChanged_) {
    sender_->uart_.begin(sender_->breakBaud_, sender_->breakFormat_);
    breakSerialParams_.getFrom(serialIndex_, port_,
                               (sender_->breakFormat_ & 0x80) != 0);
    breakSerialParamsChanged_ = false;
  }
  if (!slotsSerialParamsSet_) {
    sender_->uart_.begin(kSlotsBaud, kSlotsFormat);
    slotsSerialParams_.getFrom(serialIndex_, port_, (kSlotsFormat & 0x80) != 0);
    slotsSerialParamsSet_ = true;
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

void UARTSendHandler::setInactive() const {
  port_->C2 = UART_C2_TX_INACTIVE;
}

void UARTSendHandler::setCompleting() const {
  port_->C2 = UART_C2_TX_COMPLETING;
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

void UARTSendHandler::breakTimerCallback() const {
  if (sender_->state_ == Sender::XmitStates::kBreak) {
    port_->C3 &= ~UART_C3_TXINV;
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

void UARTSendHandler::breakTimerPreCallback() const {
  // Invert the line as close as possible to the timer start
  port_->C3 |= UART_C3_TXINV;
  setInactive();
  sender_->breakStartTime_ = micros();
}

void UARTSendHandler::rateTimerCallback() const {
  sender_->intervalTimer_.end();
  setActive();
}

void UARTSendHandler::irqHandler() const {
  uint8_t status = port_->S1;
  uint8_t control = port_->C2;

  // If the transmit buffer is empty
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {
    switch (sender_->state_) {
      case Sender::XmitStates::kBreak:
        if (sender_->breakUseTimer_ &&
            sender_->intervalTimer_.begin([this]() { breakTimerCallback(); },
                                          sender_->adjustedBreakTime_)) {
          breakTimerPreCallback();
        } else {
          // Not using a timer or starting it failed;
          // revert to the original way
          breakSerialParams_.apply(serialIndex_, port_);
          port_->D = 0;
          setCompleting();
          sender_->breakStartTime_ = micros();
        }
        break;

      case Sender::XmitStates::kMAB:  // Shouldn't be needed
        sender_->state_ = Sender::XmitStates::kData;
        setActive();
        break;

      case Sender::XmitStates::kData:
#if defined(KINETISK)
        if (fifoSize_ > 1) {
          do {
            if (sender_->inactiveBufIndex_ >= sender_->inactivePacketSize_) {
              setCompleting();
              break;
            }
            port_->S1;
            port_->D = sender_->inactiveBuf_[sender_->inactiveBufIndex_++];
          } while (port_->TCFIFO < fifoSize_);  // Transmit Count
        } else {  // No FIFO
          if (sender_->inactiveBufIndex_ >= sender_->inactivePacketSize_) {
            setCompleting();
          } else {
            port_->D = sender_->inactiveBuf_[sender_->inactiveBufIndex_++];
            if (sender_->inactiveBufIndex_ >= sender_->inactivePacketSize_) {
              setCompleting();
            }
          }
        }
#else  // No FIFO
        if (sender_->inactiveBufIndex_ >= sender_->inactivePacketSize_) {
          port_->C2 = UART_C2_TX_COMPLETING;
        } else {
          port_->D = sender_->inactiveBuf_[sender_->inactiveBufIndex_++];
          if (sender_->inactiveBufIndex_ >= sender_->inactivePacketSize_) {
            port_->C2 = UART_C2_TX_COMPLETING;
          }
        }
#endif
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
        uint32_t timeSinceBreak = micros() - sender_->breakStartTime_;
        if (timeSinceBreak < sender_->breakToBreakTime_) {
          setInactive();
          if (sender_->breakToBreakTime_ != UINT32_MAX) {
            // Non-infinite break time
            if (!sender_->intervalTimer_.begin(
                    [this]() { rateTimerCallback(); },
                    sender_->breakToBreakTime_ - timeSinceBreak)) {
              // If starting the timer failed
              setActive();
            }
          }
        } else {
          // No delay necessary
          setActive();
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
    setActive();
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
