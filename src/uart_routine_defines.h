// Putting #defines inside a header file is against the Google C++ Style Guide,
// however, there's so much duplicated UART code otherwise.

#ifndef UART_ROUTINE_DEFINES_H_
#define UART_ROUTINE_DEFINES_H_

#define UART_TX_DATA_STATE_WITH_FIFO(N)                              \
  do {                                                               \
    if (instance->outputBufIndex_ >= instance->packetSize_) {        \
      UART##N##_C2 = UART_C2_TX_COMPLETING;                          \
      break;                                                         \
    }                                                                \
    status = UART##N##_S1;                                           \
    UART##N##_D = instance->outputBuf_[instance->outputBufIndex_++]; \
  } while (UART##N##_TCFIFO < 8);

#define UART_TX_DATA_STATE_NO_FIFO(CTRL, DATA, CTRL_PREFIX)   \
  if (instance->outputBufIndex_ >= instance->packetSize_) {   \
    CTRL = CTRL_PREFIX##_TX_COMPLETING;                       \
  } else {                                                    \
    DATA = instance->outputBuf_[instance->outputBufIndex_++]; \
    if (instance->outputBufIndex_ >= instance->packetSize_) { \
      CTRL = CTRL_PREFIX##_TX_COMPLETING;                     \
    }                                                         \
  }

// Assumes status = UARTx_S1 and control = UARTx_C2 (or equivalent)
// Needs to have UART_TX_DATA_STATE_N defined.
#define UART_TX(N, CTRL, DATA, CTRL_PREFIX, STAT_PREFIX)              \
  /* If the transmit buffer is empty */                               \
  if ((control & CTRL_PREFIX##_TIE) != 0 &&                           \
      (status & STAT_PREFIX##_TDRE) != 0) {                           \
    switch (instance->state_) {                                       \
      case Sender::XmitStates::kBreak:                                \
        DATA = 0;                                                     \
        instance->timeSinceBreak_ = 0;                                \
        CTRL = CTRL_PREFIX##_TX_COMPLETING;                           \
        break;                                                        \
                                                                      \
      case Sender::XmitStates::kData:                                 \
        UART_TX_DATA_STATE_##N                                        \
        break;                                                        \
                                                                      \
      case Sender::XmitStates::kIdle:                                 \
        /* Pause management */                                        \
        if (instance->paused_) {                                      \
          CTRL = CTRL_PREFIX##_TX_INACTIVE;                           \
          return;                                                     \
        }                                                             \
        if (instance->resumeCounter_ > 0) {                           \
          if (--instance->resumeCounter_ == 0) {                      \
            instance->paused_ = true;                                 \
          }                                                           \
        }                                                             \
                                                                      \
        instance->transmitting_ = true;                               \
        instance->state_ = Sender::XmitStates::kBreak;                \
        instance->uart_.begin(kBreakBaud, kBreakFormat);              \
                                                                      \
        /* Delay so that we can achieve the specified refresh rate */ \
        uint32_t timeSinceBreak = instance->timeSinceBreak_;          \
        if (timeSinceBreak < instance->breakToBreakTime_) {           \
          CTRL = CTRL_PREFIX##_TX_INACTIVE;                           \
          if (instance->breakToBreakTime_ != UINT32_MAX) {            \
            /* Non-infinite break time */                             \
            if (!instance->refreshRateTimer_.begin(                   \
                    []() {                                            \
                      txInstances[N]->refreshRateTimer_.end();        \
                      CTRL = CTRL_PREFIX##_TX_ACTIVE;                 \
                    },                                                \
                    instance->breakToBreakTime_ - timeSinceBreak)) {  \
              /* If starting the timer failed */                      \
              CTRL = CTRL_PREFIX##_TX_ACTIVE;                         \
            }                                                         \
          }                                                           \
        } else {                                                      \
          /* No delay necessary */                                    \
          CTRL = CTRL_PREFIX##_TX_ACTIVE;                             \
        }                                                             \
        break;                                                        \
    }                                                                 \
  }

// Assumes status = UARTx_S1 and control = UARTx_C2 (or equivalent)
#define UART_TX_COMPLETE(CTRL, CTRL_PREFIX, STAT_PREFIX) \
  /* If transmission is complete */                      \
  if ((control & CTRL_PREFIX##_TCIE) != 0 &&             \
      (status & STAT_PREFIX##_TC) != 0) {                \
    switch (instance->state_) {                          \
      case Sender::XmitStates::kBreak:                   \
        instance->state_ = Sender::XmitStates::kData;    \
        instance->uart_.begin(kSlotsBaud, kSlotsFormat); \
        break;                                           \
                                                         \
      case Sender::XmitStates::kData:                    \
        instance->completePacket();                      \
        break;                                           \
      case Sender::XmitStates::kIdle:                    \
        break;                                           \
    }                                                    \
    CTRL = CTRL_PREFIX##_TX_ACTIVE;                      \
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
      instance->checkPacketTimeout();                                      \
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

// Assumes status = UARTx_S1.
// Needs to have UART_RX_CLEAR_IDLE_N defined.
#define UART_RX_NO_FIFO(N, STAT_PREFIX, DATA)      \
  /* If the receive buffer is full */              \
  if ((status & STAT_PREFIX##_RDRF) != 0) {        \
    b = DATA;                                      \
    instance->receiveByte(b);                      \
  } else if ((status & STAT_PREFIX##_IDLE) != 0) { \
    UART_RX_CLEAR_IDLE_##N                         \
    instance->checkPacketTimeout();                \
  }

// Needs to have UART_RX_N defined.
#define UART_RX(N) UART_RX_##N

#define UART_RX_ERROR_FLUSH_FIFO(N)  \
  /* Flush anything in the buffer */ \
  uint8_t avail = UART##N##_RCFIFO;  \
  if (avail > 1) {                   \
    while (--avail > 0) {            \
      b = UART##N##_D;               \
      instance->receiveByte(b);      \
    }                                \
  }

#define UART_RX_ERROR_PROCESS(DATA)    \
  b = DATA;                            \
  if (b == 0) {                        \
    instance->receivePotentialBreak(); \
  } else {                             \
    instance->receiveBadBreak();       \
  }

// Needs to have UART_RX_ERROR_FLUSH_FIFO_N defined.
#define UART_RX_ERROR(N, STAT, STAT_PREFIX, DATA)                          \
  /* A framing error likely indicates a break */                           \
  if ((STAT & STAT_PREFIX##_FE) != 0) {                                    \
    /* Only allow a packet whose framing error actually indicates a break. \
     * A value of zero indicates a true break and not some other           \
     * framing error. */                                                   \
    /* Note: Reading a byte clears interrupt flags */                      \
                                                                           \
    instance->feStartTime_ = micros();                                     \
                                                                           \
    UART_RX_ERROR_FLUSH_FIFO_##N                                           \
                                                                           \
    UART_RX_ERROR_PROCESS(DATA)                                            \
  }

#endif  // UART_ROUTINE_DEFINES_H_
