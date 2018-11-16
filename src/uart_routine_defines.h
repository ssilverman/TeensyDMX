#ifndef UART_ROUTINES_H_
#define UART_ROUTINES_H_

// Assumes status = UARTx_S1 and control = UARTx_C2
#define UART_TX_WITH_FIFO(N)                                               \
  /* If the transmit buffer is empty */                                    \
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {      \
    switch (instance->state_) {                                            \
      case Sender::XmitStates::kBreak:                                     \
        UART##N##_D = 0;                                                   \
        instance->timeSinceBreak_ = 0;                                     \
        UART##N##_C2 = UART_C2_TX_COMPLETING;                              \
        break;                                                             \
                                                                           \
      case Sender::XmitStates::kData:                                      \
        do {                                                               \
          if (instance->outputBufIndex_ >= instance->packetSize_) {        \
            instance->completePacket();                                    \
            UART##N##_C2 = UART_C2_TX_COMPLETING;                          \
            break;                                                         \
          }                                                                \
          status = UART##N##_S1;                                           \
          UART##N##_D = instance->outputBuf_[instance->outputBufIndex_++]; \
        } while (UART##N##_TCFIFO < 8);                                    \
        break;                                                             \
                                                                           \
      case Sender::XmitStates::kIdle:                                      \
        instance->state_ = Sender::XmitStates::kBreak;                     \
        instance->uart_.begin(kBreakBaud, kBreakFormat);                   \
                                                                           \
        /* Delay so that we can achieve the specified refresh rate */      \
        uint32_t timeSinceBreak = instance->timeSinceBreak_;               \
        if (timeSinceBreak < instance->breakToBreakTime_) {                \
          UART##N##_C2 = UART_C2_TX_INACTIVE;                              \
          if (instance->breakToBreakTime_ != UINT32_MAX) {                 \
            /* Non-infinite break time */                                  \
            if (!instance->timer_.begin(                                   \
                    []() {                                                 \
                      txInstances[N]->timer_.end();                        \
                      UART##N##_C2 = UART_C2_TX_ACTIVE;                    \
                    },                                                     \
                    instance->breakToBreakTime_ - timeSinceBreak)) {       \
              /* If starting the timer failed */                           \
              UART##N##_C2 = UART_C2_TX_ACTIVE;                            \
            }                                                              \
          }                                                                \
        } else {                                                           \
          /* No delay necessary */                                         \
          UART##N##_C2 = UART_C2_TX_ACTIVE;                                \
        }                                                                  \
        break;                                                             \
    }                                                                      \
  }

// Assumes status = UARTx_S1 and control = UARTx_C2
#define UART_TX_NO_FIFO(N)                                                 \
  /* If the transmit buffer is empty */                                    \
  if ((control & UART_C2_TIE) != 0 && (status & UART_S1_TDRE) != 0) {      \
    switch (instance->state_) {                                            \
      case Sender::XmitStates::kBreak:                                     \
        UART##N##_D = 0;                                                   \
        instance->timeSinceBreak_ = 0;                                     \
        UART##N##_C2 = UART_C2_TX_COMPLETING;                              \
        break;                                                             \
                                                                           \
      case Sender::XmitStates::kData:                                      \
        if (instance->outputBufIndex_ >= instance->packetSize_) {          \
          instance->completePacket();                                      \
          UART##N##_C2 = UART_C2_TX_COMPLETING;                            \
        } else {                                                           \
          UART##N##_D = instance->outputBuf_[instance->outputBufIndex_++]; \
          if (instance->outputBufIndex_ >= instance->packetSize_) {        \
            instance->completePacket();                                    \
            UART##N##_C2 = UART_C2_TX_COMPLETING;                          \
          }                                                                \
        }                                                                  \
        break;                                                             \
                                                                           \
      case Sender::XmitStates::kIdle:                                      \
        instance->state_ = Sender::XmitStates::kBreak;                     \
        instance->uart_.begin(kBreakBaud, kBreakFormat);                   \
                                                                           \
        /* Delay so that we can achieve the specified refresh rate */      \
        uint32_t timeSinceBreak = instance->timeSinceBreak_;               \
        if (timeSinceBreak < instance->breakToBreakTime_) {                \
          UART##N##_C2 = UART_C2_TX_INACTIVE;                              \
          if (instance->breakToBreakTime_ != UINT32_MAX) {                 \
            /* Non-infinite break time */                                  \
            if (!instance->timer_.begin(                                   \
                    []() {                                                 \
                      txInstances[N]->timer_.end();                        \
                      UART##N##_C2 = UART_C2_TX_ACTIVE;                    \
                    },                                                     \
                    instance->breakToBreakTime_ - timeSinceBreak)) {       \
              /* If starting the timer failed */                           \
              UART##N##_C2 = UART_C2_TX_ACTIVE;                            \
            }                                                              \
          }                                                                \
        } else {                                                           \
          /* No delay necessary */                                         \
          UART##N##_C2 = UART_C2_TX_ACTIVE;                                \
        }                                                                  \
        break;                                                             \
    }                                                                      \
  }

// Assumes status = UARTx_S1 and control = UARTx_C2
#define UART_TX_COMPLETE(N)                                          \
  /* If transmission is complete */                                  \
  if ((control & UART_C2_TCIE) != 0 && (status & UART_S1_TC) != 0) { \
    switch (instance->state_) {                                      \
      case Sender::XmitStates::kBreak:                               \
        instance->state_ = Sender::XmitStates::kData;                \
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);             \
        break;                                                       \
                                                                     \
      case Sender::XmitStates::kData:                                \
      case Sender::XmitStates::kIdle:                                \
        break;                                                       \
    }                                                                \
    UART##N##_C2 = UART_C2_TX_ACTIVE;                                \
  }

// Assumes status = UARTx_S1
#define UART_RX_WITH_FIFO(N)                                               \
  /* If the receive buffer is full or there's an idle condition */         \
  if ((status & (UART_S1_RDRF | UART_S1_IDLE)) != 0) {                     \
    __disable_irq();                                                       \
    uint8_t avail = UART##N##_RCFIFO;                                      \
    if (avail == 0) {                                                      \
      /* Read the register to clear the interrupt, but since it's empty,   \
       * this causes the FIFO to become misaligned, so send RXFLUSH to     \
       * reinitialize its pointers.                                        \
       * Do this inside no interrupts to avoid a potential race condition  \
       * between reading RCFIFO and flushing the FIFO. */                  \
      b = UART##N##_D;                                                     \
      UART##N##_CFIFO = UART_CFIFO_RXFLUSH;                                \
      __enable_irq();                                                      \
      return;                                                              \
    } else {                                                               \
      __enable_irq();                                                      \
      /* Read all but the last available, then read S1 and the final value \
       * So says the chip docs,                                            \
       * Section 47.3.5 UART Status Register 1 (UART_S1)                   \
       * In the NOTE part. */                                              \
      while (--avail > 0) {                                                \
        b = UART##N##_D;                                                   \
        instance->receiveByte(b);                                          \
      }                                                                    \
      status = UART##N##_S1;                                               \
      b = UART##N##_D;                                                     \
      instance->receiveByte(b);                                            \
    }                                                                      \
  }

// Assumes status = UARTx_S1
#define UART_RX_NO_FIFO(N)            \
  /* If the receive buffer is full */ \
  if ((status & UART_S1_RDRF) != 0) { \
    b = UART##N##_D;                  \
    instance->receiveByte(b);         \
  }

#define UART_RX_ERROR_FLUSH_FIFO(N)   \
  /* Flush anything in the buffer */ \
  uint8_t avail = UART##N##_RCFIFO;  \
  if (avail > 1) {                   \
    while (--avail > 0) {            \
      b = UART##N##_D;               \
      instance->receiveByte(b);      \
    }                                \
  }

// Assumes b = UARTx_D or b = LPUARTx_DATA
#define UART_RX_ERROR_PROCESS       \
  if (b == 0) {                     \
    instance->receiveBreak();       \
  } else {                          \
    /* Not a break */               \
    instance->framingErrorCount_++; \
    /* Don't keep the packet        \
     * See: [BREAK timing at the receiver](http://www.rdmprotocol.org/forums/showthread.php?t=1292) */ \
    instance->activeBufIndex_ = 0;  \
    instance->completePacket();     \
  }

#endif  // UART_ROUTINES_H_
