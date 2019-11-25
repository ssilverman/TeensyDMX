// This file is part of the TeensyDMX library.
// (c) 2018-2019 Shawn Silverman

// Putting #defines inside a header file is against the Google C++ Style Guide,
// however, there's so much duplicated UART code otherwise.

#ifndef UART_ROUTINE_DEFINES_H_
#define UART_ROUTINE_DEFINES_H_

// ---------------------------------------------------------------------------
//  UART TX routines, for Sender
// ---------------------------------------------------------------------------

#define UART_TX_DATA_STATE_WITH_FIFO(N)                              \
  do {                                                               \
    if (instance->outputBufIndex_ >= instance->packetSize_) {        \
      UART##N##_C2 = UART_C2_TX_COMPLETING;                          \
      break;                                                         \
    }                                                                \
    status = UART##N##_S1;                                           \
    UART##N##_D = instance->outputBuf_[instance->outputBufIndex_++]; \
  } while (UART##N##_TCFIFO < 8);

#define UART_TX_DATA_STATE_NO_FIFO(N)                                \
  if (instance->outputBufIndex_ >= instance->packetSize_) {          \
    UART##N##_C2 = UART_C2_TX_COMPLETING;                            \
  } else {                                                           \
    UART##N##_D = instance->outputBuf_[instance->outputBufIndex_++]; \
    if (instance->outputBufIndex_ >= instance->packetSize_) {        \
      UART##N##_C2 = UART_C2_TX_COMPLETING;                          \
    }                                                                \
  }

#define LPUART_TX_DATA_STATE_NO_FIFO(N)                                   \
  if (instance->outputBufIndex_ >= instance->packetSize_) {               \
    LPUART##N##_CTRL = LPUART_CTRL_TX_COMPLETING;                         \
  } else {                                                                \
    LPUART##N##_DATA = instance->outputBuf_[instance->outputBufIndex_++]; \
    if (instance->outputBufIndex_ >= instance->packetSize_) {             \
      LPUART##N##_CTRL = LPUART_CTRL_TX_COMPLETING;                       \
    }                                                                     \
  }

// N is the register number.
#define LPUART_TX_DATA_STATE_WITH_FIFO(N)                                 \
  do {                                                                    \
    if (instance->outputBufIndex_ >= instance->packetSize_) {             \
      LPUART##N##_CTRL = LPUART_CTRL_TX_COMPLETING;                       \
      break;                                                              \
    }                                                                     \
    status = LPUART##N##_STAT;                                            \
    LPUART##N##_DATA = instance->outputBuf_[instance->outputBufIndex_++]; \
  } while (((LPUART##N##_WATER >> 8) & 0x07) < 4);

// Assumes status = UARTx_S1 and control = UARTx_C2 (or equivalent).
// Needs to have UART_TX_DATA_STATE_REG defined.
// Needs to have REATTACH_INTERRUPT_REG defined.
#define UART_TX(INSTANCE, REG, CTRL, DATA, CTRL_PREFIX, STAT_PREFIX)  \
  Sender *instance = txInstances[INSTANCE];                           \
  if (instance == nullptr) {                                          \
    return;                                                           \
  }                                                                   \
                                                                      \
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
        UART_TX_DATA_STATE_##REG                                      \
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
        REATTACH_INTERRUPT_##REG                                      \
                                                                      \
        /* Delay so that we can achieve the specified refresh rate */ \
        uint32_t timeSinceBreak = instance->timeSinceBreak_;          \
        if (timeSinceBreak < instance->breakToBreakTime_) {           \
          CTRL = CTRL_PREFIX##_TX_INACTIVE;                           \
          if (instance->breakToBreakTime_ != UINT32_MAX) {            \
            /* Non-infinite break time */                             \
            if (!instance->refreshRateTimer_.begin(                   \
                    []() {                                            \
                      Sender *s = txInstances[INSTANCE];              \
                      if (s != nullptr) {                             \
                        s->refreshRateTimer_.end();                   \
                        CTRL = CTRL_PREFIX##_TX_ACTIVE;               \
                      }                                               \
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

// Assumes status = UARTx_S1 and control = UARTx_C2 (or equivalent).
// Assumes instance is defined.
// Needs to have REATTACH_INTERRUPT_REG defined.
#define UART_TX_COMPLETE(REG, CTRL, CTRL_PREFIX, STAT_PREFIX) \
  /* If transmission is complete */                           \
  if ((control & CTRL_PREFIX##_TCIE) != 0 &&                  \
      (status & STAT_PREFIX##_TC) != 0) {                     \
    switch (instance->state_) {                               \
      case Sender::XmitStates::kBreak:                        \
        instance->state_ = Sender::XmitStates::kData;         \
        instance->uart_.begin(kSlotsBaud, kSlotsFormat);      \
        REATTACH_INTERRUPT_##REG                              \
        break;                                                \
                                                              \
      case Sender::XmitStates::kData:                         \
        instance->completePacket();                           \
        break;                                                \
      case Sender::XmitStates::kIdle:                         \
        break;                                                \
    }                                                         \
    CTRL = CTRL_PREFIX##_TX_ACTIVE;                           \
  }

// ---------------------------------------------------------------------------
//  UART RX routines, for Receiver
// ---------------------------------------------------------------------------

// Assumes status = UARTx_S1.
// Needs to have UART_RX_TEST_FIRST_STOP_BIT_N defined.
// N is the register number.
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
      UART##N##_D;                                                         \
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
      bool errFlag = false;                                                \
      while (--avail > 0) {                                                \
        /* Check that the 9th bit is high; used as the first stop bit */   \
        if (!errFlag && !UART_RX_TEST_FIRST_STOP_BIT_##N) {                \
          errFlag = true;                                                  \
          instance->framingErrorCount_++;                                  \
          instance->completePacket();                                      \
        }                                                                  \
        instance->receiveByte(UART##N##_D);                                \
      }                                                                    \
      status = UART##N##_S1;                                               \
      if (!errFlag && !UART_RX_TEST_FIRST_STOP_BIT_##N) {                  \
        instance->framingErrorCount_++;                                    \
        instance->completePacket();                                        \
      }                                                                    \
      instance->receiveByte(UART##N##_D);                                  \
    }                                                                      \
  }

// Assumes status = LPUARTx_STAT.
// N is the register number.
#define LPUART_RX_WITH_FIFO(N)                                     \
  /* If the receive buffer is full or there's an idle condition */ \
  if ((status & (LPUART_STAT_RDRF | LPUART_STAT_IDLE)) != 0) {     \
    uint8_t avail = (LPUART##N##_WATER >> 24) & 0x07;              \
    if (avail == 0) {                                              \
      instance->checkPacketTimeout();                              \
      if ((status & LPUART_STAT_IDLE) != 0) {                      \
        LPUART##N##_STAT |= LPUART_STAT_IDLE;                      \
      }                                                            \
      return;                                                      \
    } else {                                                       \
      while (avail-- > 0) {                                        \
        instance->receiveByte(LPUART##N##_DATA);                   \
      }                                                            \
    }                                                              \
  }

// Assumes status = UARTx_S1.
// Needs to have UART_RX_TEST_FIRST_STOP_BIT_N defined.
// Needs to have UART_RX_CLEAR_IDLE_N defined.
// N is the register number.
#define UART_RX_NO_FIFO(N)                                           \
  /* If the receive buffer is full */                                \
  if ((status & UART_S1_RDRF) != 0) {                                \
    /* Check that the 9th bit is high; used as the first stop bit */ \
    if (!UART_RX_TEST_FIRST_STOP_BIT_##N) {                          \
      instance->framingErrorCount_++;                                \
      instance->completePacket();                                    \
    }                                                                \
    instance->receiveByte(UART##N##_D);                              \
  } else if ((status & UART_S1_IDLE) != 0) {                         \
    instance->checkPacketTimeout();                                  \
    UART_RX_CLEAR_IDLE_##N                                           \
  }

// Assumes status = LPUARTy_STAT.
// N is the register number.
#define LPUART_RX_NO_FIFO(N)                     \
  /* If the receive buffer is full */            \
  if ((status & LPUART_STAT_RDRF) != 0) {        \
    instance->receiveByte(LPUART##N##_DATA);     \
  } else if ((status & LPUART_STAT_IDLE) != 0) { \
    instance->checkPacketTimeout();              \
    LPUART##N##_STAT |= LPUART_STAT_IDLE;        \
  }

// Assumes status = UARTx_S1 or LPUARTy_STAT.
// Needs to have UART_RX_CLEAR_ERRORS_REG defined.
// Needs to have UART_RX_ERROR_FLUSH_FIFO_REG defined.
// Needs to have UART_RX_REG defined.
// DATA may be a 32-bit register with non-data bits.
#define UART_RX(INSTANCE, REG, STAT_PREFIX, DATA)                          \
  Receiver *instance = rxInstances[INSTANCE];                              \
  if (instance == nullptr) {                                               \
    return;                                                                \
  }                                                                        \
                                                                           \
  /* A framing error likely indicates a break */                           \
  if ((status & STAT_PREFIX##_FE) != 0) {                                  \
    /* Only allow a packet whose framing error actually indicates a break. \
     * A value of zero indicates a true break and not some other           \
     * framing error. */                                                   \
                                                                           \
    instance->feStartTime_ = micros();                                     \
                                                                           \
    UART_RX_CLEAR_ERRORS_##REG                                             \
    UART_RX_ERROR_FLUSH_FIFO_##REG                                         \
                                                                           \
    /* DATA may be a 32-bit register with extra bits */                    \
    if ((DATA & 0xff) == 0) {                                              \
      instance->receivePotentialBreak();                                   \
    } else {                                                               \
      instance->receiveBadBreak();                                         \
    }                                                                      \
    return;                                                                \
  }                                                                        \
                                                                           \
  UART_RX_##REG

// N is the register number.
#define UART_RX_ERROR_FLUSH_FIFO(N)       \
  /* Flush anything in the buffer */      \
  uint8_t avail = UART##N##_RCFIFO;       \
  if (avail > 1) {                        \
    while (--avail > 0) {                 \
      instance->receiveByte(UART##N##_D); \
    }                                     \
  }

// ---------------------------------------------------------------------------
//  Synchronous TX routines, for Receiver
// ---------------------------------------------------------------------------

// Synchronous TX, used in Receiver.
// Needs to have UART_SYNC_TX_SEND_FIFO_N defined.
// N is the register number.
#define UART_SYNC_TX(N, STAT, STAT_PREFIX, DATA) \
  if (len <= 0) {                                \
    return;                                      \
  }                                              \
                                                 \
  while (len > 0) {                              \
    while ((STAT & STAT_PREFIX##_TDRE) == 0) {   \
      /* Wait until we can transmit */           \
    }                                            \
    DATA = *(b++);                               \
    len--;                                       \
                                                 \
    UART_SYNC_TX_SEND_FIFO_##N                   \
  }                                              \
                                                 \
  while ((STAT & STAT_PREFIX##_TC) == 0) {       \
    /* Wait until transmission complete */       \
  }

// N is the register number.
#define UART_SYNC_TX_SEND_FIFO(N)           \
  while (len > 0 && UART##N##_TCFIFO < 8) { \
    UART##N##_S1;                           \
    UART##N##_D = *(b++);                   \
    len--;                                  \
  }

// N is the register number.
#define LPUART_SYNC_TX_SEND_FIFO(N)                          \
  while (len > 0 && ((LPUART##N##_WATER >> 8) & 0x07) < 4) { \
    LPUART##N##_DATA = *(b++);                               \
    len--;                                                   \
  }

// N is the register number.
#define UART_TX_FLUSH_FIFO(N)        \
  while (UART##N##_TCFIFO > 0) {     \
    /* Wait for the FIFO to drain */ \
  }

// Needs to have UART_TX_FLUSH_FIFO_N defined.
// N is the register number.
#define UART_TX_BREAK(N)                                    \
  if (count <= 0) {                                         \
    return;                                                 \
  }                                                         \
                                                            \
  UART_TX_FLUSH_FIFO_##N                                    \
                                                            \
  while ((UART##N##_S1 & UART_S1_TDRE) == 0) {              \
    /* Wait until we can transmit */                        \
  }                                                         \
  UART##N##_C2 |= UART_C2_SBK;  /* Enable BREAK transmit */ \
  /* Turn off in the middle of the last BREAK */            \
  delayMicroseconds((count - 1)*kCharTime + kBitTime);      \
  UART##N##_C2 &= ~UART_C2_SBK;                             \
                                                            \
  /* Account for the shift register time by 1 character;    \
   * this overlaps */                                       \
  delayMicroseconds(kCharTime - kBitTime + mabTime + 1);

// N is the register number.
#define LPUART_TX_BREAK(N)                                  \
  if (count <= 0) {                                         \
    return;                                                 \
  }                                                         \
                                                            \
  while (count-- > 0) {                                     \
    while ((LPUART##N##_STAT & LPUART_STAT_TDRE) == 0) {    \
      /* Wait until we can transmit*/                       \
    }                                                       \
    LPUART##N##_DATA = LPUART_DATA_FRETSC; /* T9 is zero */ \
  }                                                         \
                                                            \
  delayMicroseconds(mabTime);

#endif  // UART_ROUTINE_DEFINES_H_
