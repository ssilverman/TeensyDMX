// This file is part of the TeensyDMX library.
// (c) 2018-2020 Shawn Silverman

// Putting #defines inside a header file is against the Google C++ Style Guide,
// however, there's so much duplicated UART code otherwise.

#ifndef TEENSYDMX_UART_ROUTINE_DEFINES_H_
#define TEENSYDMX_UART_ROUTINE_DEFINES_H_

// ---------------------------------------------------------------------------
//  UART TX routines, for Sender
// ---------------------------------------------------------------------------

// Sends data to a UART with a FIFO. Used inside UART_TX
// as UART_TX_DATA_STATE_REG.
// N is the register number.
#define UART_TX_DATA_STATE_WITH_FIFO(N)                              \
  do {                                                               \
    if (instance->outputBufIndex_ >= instance->packetSize_) {        \
      UART##N##_C2 = UART_C2_TX_COMPLETING;                          \
      break;                                                         \
    }                                                                \
    status = UART##N##_S1;                                           \
    UART##N##_D = instance->outputBuf_[instance->outputBufIndex_++]; \
  } while (UART##N##_TCFIFO < 8);

// Sends data to a UART with no FIFO. Used inside UART_TX
// as UART_TX_DATA_STATE_REG.
// N is the register number.
#define UART_TX_DATA_STATE_NO_FIFO(N)                                \
  if (instance->outputBufIndex_ >= instance->packetSize_) {          \
    UART##N##_C2 = UART_C2_TX_COMPLETING;                            \
  } else {                                                           \
    UART##N##_D = instance->outputBuf_[instance->outputBufIndex_++]; \
    if (instance->outputBufIndex_ >= instance->packetSize_) {        \
      UART##N##_C2 = UART_C2_TX_COMPLETING;                          \
    }                                                                \
  }

// Sends data to an LPUART with no FIFO. Used inside UART_TX
// as UART_TX_DATA_STATE_REG.
// N is the register number.
#define LPUART_TX_DATA_STATE_NO_FIFO(N)                                   \
  if (instance->outputBufIndex_ >= instance->packetSize_) {               \
    LPUART##N##_CTRL = LPUART_CTRL_TX_COMPLETING;                         \
  } else {                                                                \
    LPUART##N##_DATA = instance->outputBuf_[instance->outputBufIndex_++]; \
    if (instance->outputBufIndex_ >= instance->packetSize_) {             \
      LPUART##N##_CTRL = LPUART_CTRL_TX_COMPLETING;                       \
    }                                                                     \
  }

// Sends data to an LPUART with a FIFO. Used inside UART_TX
// as UART_TX_DATA_STATE_REG.
// N is the register number.
#define LPUART_TX_DATA_STATE_WITH_FIFO(N)                                 \
  do {                                                                    \
    if (instance->outputBufIndex_ >= instance->packetSize_) {             \
      LPUART##N##_CTRL = LPUART_CTRL_TX_COMPLETING;                       \
      break;                                                              \
    }                                                                     \
    LPUART##N##_DATA = instance->outputBuf_[instance->outputBufIndex_++]; \
  } while (((LPUART##N##_WATER >> 8) & 0x07) < 4);

// Sets the baud rate for a UART inside a KINETISK chip. Used inside UART_TX as
// UART_TX_SET_BREAK_BAUD_REG and inside UART_TX_COMPLETE
// as UART_TX_SET_SLOTS_BAUD_REG.
// N is the register number.
#define KINETISK_SET_BAUD(N, DIV, FORMAT_FUNC) \
  { /* New block because defining */           \
    /* If DIV < 32 DIV = 32 */                 \
    uint32_t div = (DIV);                      \
    if (div < 32) {                            \
      div = 32;                                \
    }                                          \
    UART##N##_BDH = (div >> 13) & 0x1f;        \
    UART##N##_BDL = (div >> 5) & 0xff;         \
    UART##N##_C4 = div & 0x1f;                 \
    FORMAT_FUNC;                               \
  }

// Sets the baud rate for a UART inside a KINETISL chip. Used inside UART_TX as
// UART_TX_SET_BREAK_BAUD_REG and inside UART_TX_COMPLETE
// as UART_TX_SET_SLOTS_BAUD_REG.
// TWO_STOP_BITS is a bool.
// N is the register number.
#define KINETISL_SET_BAUD(N, DIV, TWO_STOP_BITS) \
  { /* New block because defining */             \
    /* If DIV < 1 DIV = 1 */                     \
    uint32_t div = (DIV);                        \
    if (div < 1) {                               \
      div = 1;                                   \
    }                                            \
    UART##N##_BDH = (div >> 8) & 0x1f;           \
    if ((TWO_STOP_BITS)) {                       \
      UART##N##_BDH |= UART_BDH_SBNS;            \
    }                                            \
    UART##N##_BDL = div & 0xff;                  \
  }

// Sets the baud rate for an LPUART. Used inside UART_TX as
// UART_TX_SET_BREAK_BAUD_REG and inside UART_TX_COMPLETE
// as UART_TX_SET_SLOTS_BAUD_REG.
// N is the register number.
#define LPUART_SET_BAUD(N, PARAMS_MEMBER)          \
  LPUART##N##_BAUD = instance->PARAMS_MEMBER.baud; \
  LPUART##N##_STAT = instance->PARAMS_MEMBER.stat; \
  LPUART##N##_CTRL = instance->PARAMS_MEMBER.ctrl;

// Transmit routine for a UART and LPUART.
// Assumes status = UARTx_S1 and control = UARTx_C2 (or equivalent).
// Needs to have UART_TX_DATA_STATE_REG defined.
// Needs to have UART_TX_SET_BREAK_BAUD_REG defined.
#define UART_TX(INSTANCE, REG, CTRL, CTRLINV, DATA, CTRL_PREFIX,        \
                CTRLINV_PREFIX, STAT_PREFIX)                            \
  Sender *instance = txInstances[INSTANCE];                             \
  if (instance == nullptr) {                                            \
    return;                                                             \
  }                                                                     \
                                                                        \
  /* If the transmit buffer is empty */                                 \
  if ((control & CTRL_PREFIX##_TIE) != 0 &&                             \
      (status & STAT_PREFIX##_TDRE) != 0) {                             \
    switch (instance->state_) {                                         \
      case Sender::XmitStates::kBreak:                                  \
        if (!instance->periodicTimer_.begin(                            \
                [instance]() {                                          \
                  if (instance->state_ == Sender::XmitStates::kBreak) { \
                    CTRLINV &= ~CTRLINV_PREFIX##_TXINV;                 \
                    instance->state_ = Sender::XmitStates::kMAB;        \
                    if (instance->periodicTimer_.restart(               \
                            instance->adjustedMABTime_)) {              \
                      return;                                           \
                    }                                                   \
                    /* We shouldn't delay as an alternative because     \
                     * that might mean we delay too long */             \
                  }                                                     \
                  instance->periodicTimer_.end();                       \
                  instance->state_ = Sender::XmitStates::kData;         \
                  CTRL = CTRL_PREFIX##_TX_ACTIVE;                       \
                },                                                      \
                instance->breakTime_,                                   \
                [instance]() {                                          \
                  CTRL = CTRL_PREFIX##_TX_INACTIVE;                     \
                  /* Invert the line as close as possible to the        \
                   * interrupt start */                                 \
                  CTRLINV |= CTRLINV_PREFIX##_TXINV;                    \
                  instance->breakStartTime_ = micros();                 \
                })) {                                                   \
          /* Starting the timer failed, revert to the original way */   \
          UART_TX_SET_BREAK_BAUD_##REG                                  \
          DATA = 0;                                                     \
          CTRL = CTRL_PREFIX##_TX_COMPLETING;                           \
          instance->breakStartTime_ = micros();                         \
        }                                                               \
        break;                                                          \
                                                                        \
      case Sender::XmitStates::kMAB:  /* Shouldn't be needed */         \
        instance->state_ = Sender::XmitStates::kData;                   \
        CTRL = CTRL_PREFIX##_TX_ACTIVE;                                 \
        break;                                                          \
                                                                        \
      case Sender::XmitStates::kData:                                   \
        UART_TX_DATA_STATE_##REG                                        \
        break;                                                          \
                                                                        \
      case Sender::XmitStates::kIdle: {                                 \
        /* Pause management */                                          \
        if (instance->paused_) {                                        \
          CTRL = CTRL_PREFIX##_TX_INACTIVE;                             \
          return;                                                       \
        }                                                               \
        if (instance->resumeCounter_ > 0) {                             \
          if (--instance->resumeCounter_ == 0) {                        \
            instance->paused_ = true;                                   \
          }                                                             \
        }                                                               \
                                                                        \
        instance->transmitting_ = true;                                 \
        instance->state_ = Sender::XmitStates::kBreak;                  \
                                                                        \
        /* Delay so that we can achieve the specified refresh rate */   \
        uint32_t timeSinceBreak = micros() - instance->breakStartTime_; \
        if (timeSinceBreak < instance->breakToBreakTime_) {             \
          CTRL = CTRL_PREFIX##_TX_INACTIVE;                             \
          if (instance->breakToBreakTime_ != UINT32_MAX) {              \
            /* Non-infinite BREAK time */                               \
            if (!instance->periodicTimer_.begin(                        \
                    [instance]() {                                      \
                      instance->periodicTimer_.end();                   \
                      CTRL = CTRL_PREFIX##_TX_ACTIVE;                   \
                    },                                                  \
                    instance->breakToBreakTime_ - timeSinceBreak)) {    \
              /* If starting the timer failed */                        \
              CTRL = CTRL_PREFIX##_TX_ACTIVE;                           \
            }                                                           \
          }                                                             \
        } else {                                                        \
          /* No delay necessary */                                      \
          CTRL = CTRL_PREFIX##_TX_ACTIVE;                               \
        }                                                               \
        break;                                                          \
      }                                                                 \
                                                                        \
      default:                                                          \
        break;                                                          \
    }                                                                   \
  }

// Routine for when transmission is complete, for a UART or LPUART.
// Assumes status = UARTx_S1 and control = UARTx_C2 (or equivalent).
// Assumes instance is defined.
// Needs to have UART_TX_SET_SLOTS_BAUD_REG defined.
#define UART_TX_COMPLETE(REG, CTRL, CTRL_PREFIX, STAT_PREFIX) \
  /* If transmission is complete */                           \
  if ((control & CTRL_PREFIX##_TCIE) != 0 &&                  \
      (status & STAT_PREFIX##_TC) != 0) {                     \
    switch (instance->state_) {                               \
      case Sender::XmitStates::kBreak:                        \
        instance->state_ = Sender::XmitStates::kData;         \
        UART_TX_SET_SLOTS_BAUD_##REG                          \
        break;                                                \
                                                              \
      case Sender::XmitStates::kMAB:  /* Shouldn't need */    \
        instance->state_ = Sender::XmitStates::kData;         \
        UART_TX_SET_SLOTS_BAUD_##REG                          \
        break;                                                \
                                                              \
      case Sender::XmitStates::kData:                         \
        instance->completePacket();                           \
        break;                                                \
                                                              \
      case Sender::XmitStates::kIdle:                         \
        break;                                                \
                                                              \
      default:                                                \
        break;                                                \
    }                                                         \
    CTRL = CTRL_PREFIX##_TX_ACTIVE;                           \
  }

// ---------------------------------------------------------------------------
//  UART RX routines, for Receiver
// ---------------------------------------------------------------------------

// Receives data from a UART with a FIFO. Used inside UART_RX as UART_RX_REG.
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
      instance->receiveIdle();                                             \
    } else {                                                               \
      __enable_irq();                                                      \
      uint32_t timestamp = micros() - kCharTime*avail;                     \
      if (avail < UART##N##_RWFIFO) {                                      \
        timestamp -= kCharTime;                                            \
      }                                                                    \
      /* Read all but the last available, then read S1 and the final value \
       * So says the chip docs,                                            \
       * Section 47.3.5 UART Status Register 1 (UART_S1)                   \
       * In the NOTE part. */                                              \
      bool errFlag = false;                                                \
      while (--avail > 0) {                                                \
        /* Check that the 9th bit is high; used as the first stop bit */   \
        if (!errFlag && !UART_RX_TEST_FIRST_STOP_BIT_##N) {                \
          errFlag = true;                                                  \
          instance->receiveBadBreak();                                     \
        }                                                                  \
        instance->receiveByte(UART##N##_D, timestamp += kCharTime);        \
      }                                                                    \
      status = UART##N##_S1;                                               \
      if (!errFlag && !UART_RX_TEST_FIRST_STOP_BIT_##N) {                  \
        instance->receiveBadBreak();                                       \
      }                                                                    \
      instance->receiveByte(UART##N##_D, timestamp + kCharTime);           \
    }                                                                      \
  }

// Receives data from an LPUART with a FIFO. Used inside UART_RX as UART_RX_REG.
// Assumes status = LPUARTx_STAT.
// N is the register number.
#define LPUART_RX_WITH_FIFO(N)                                           \
  /* If the receive buffer is full or there's an idle condition */       \
  if ((status & (LPUART_STAT_RDRF | LPUART_STAT_IDLE)) != 0) {           \
    uint8_t avail = (LPUART##N##_WATER >> 24) & 0x07;                    \
    if (avail == 0) {                                                    \
      instance->receiveIdle();                                           \
      if ((status & LPUART_STAT_IDLE) != 0) {                            \
        LPUART##N##_STAT |= LPUART_STAT_IDLE;                            \
      }                                                                  \
    } else {                                                             \
      uint32_t timestamp = micros() - kCharTime*avail;                   \
      if (avail < ((LPUART##N##_WATER >> 16) & 0x03)) {                  \
        timestamp -= kCharTime;                                          \
      }                                                                  \
      while (avail-- > 0) {                                              \
        instance->receiveByte(LPUART##N##_DATA, timestamp += kCharTime); \
      }                                                                  \
    }                                                                    \
  }

// Receives data from a UART with no FIFO. Used inside UART_RX as UART_RX_REG.
// Assumes status = UARTx_S1.
// Needs to have UART_RX_TEST_FIRST_STOP_BIT_N defined.
// Needs to have UART_RX_CLEAR_IDLE_N defined.
// N is the register number.
#define UART_RX_NO_FIFO(N)                                           \
  /* If the receive buffer is full */                                \
  if ((status & UART_S1_RDRF) != 0) {                                \
    /* Check that the 9th bit is high; used as the first stop bit */ \
    if (!UART_RX_TEST_FIRST_STOP_BIT_##N) {                          \
      instance->receiveBadBreak();                                   \
    }                                                                \
    instance->receiveByte(UART##N##_D, micros());                    \
  } else if ((status & UART_S1_IDLE) != 0) {                         \
    instance->receiveIdle();                                         \
    UART_RX_CLEAR_IDLE_##N                                           \
  }

// Receives data from an LPUART with no FIFO. Used inside UART_RX
// as UART_RX_REG.
// Assumes status = LPUARTy_STAT.
// N is the register number.
#define LPUART_RX_NO_FIFO(N)                           \
  /* If the receive buffer is full */                  \
  if ((status & LPUART_STAT_RDRF) != 0) {              \
    instance->receiveByte(LPUART##N##_DATA, micros()); \
  } else if ((status & LPUART_STAT_IDLE) != 0) {       \
    instance->receiveIdle();                           \
    LPUART##N##_STAT |= LPUART_STAT_IDLE;              \
  }

// UART and LPUART RX routine.
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
  /* A framing error likely indicates a BREAK */                           \
  if ((status & STAT_PREFIX##_FE) != 0) {                                  \
    /* Only allow a packet whose framing error actually indicates a BREAK. \
     * A value of zero indicates a true BREAK and not some other           \
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

// Flushes a UART's FIFO when a framing error was detected. Used inside UART_RX
// as UART_RX_ERROR_FLUSH_FIFO_REG.
// N is the register number.
#define UART_RX_ERROR_FLUSH_FIFO(N)                               \
  /* Flush anything in the buffer */                              \
  uint8_t avail = UART##N##_RCFIFO;                               \
  if (avail > 1) {                                                \
    /* Read everything but the last byte */                       \
    uint32_t timestamp = micros() - kCharTime*avail;              \
    if (avail < UART##N##_RWFIFO) {                               \
      timestamp -= kCharTime;                                     \
    }                                                             \
    while (--avail > 0) {                                         \
      instance->receiveByte(UART##N##_D, timestamp += kCharTime); \
    }                                                             \
  }

// Flushes an LPUART's FIFO when a framing error was detected. Used inside
// UART_RX as UART_RX_ERROR_FLUSH_FIFO_REG.
// N is the register number.
#define LPUART_RX_ERROR_FLUSH_FIFO(N)                                  \
  /* Flush anything in the buffer */                                   \
  uint8_t avail = (LPUART##N##_WATER >> 24) & 0x07;                    \
  if (avail > 1) {                                                     \
    /* Read everything but the last byte */                            \
    uint32_t timestamp = micros() - kCharTime*avail;                   \
    while (--avail > 0) {                                              \
      instance->receiveByte(LPUART##N##_DATA, timestamp += kCharTime); \
    }                                                                  \
  }

// ---------------------------------------------------------------------------
//  Synchronous TX routines, for Receiver
// ---------------------------------------------------------------------------

// Synchronous TX routine for UARTs with a FIFO. Used inside UART_SYNC_TX
// as UART_SYNC_TX_SEND_FIFO_N.
// N is the register number.
#define UART_SYNC_TX_SEND_FIFO(N)           \
  while (len > 0 && UART##N##_TCFIFO < 8) { \
    UART##N##_S1;                           \
    UART##N##_D = *(b++);                   \
    len--;                                  \
  }

// Synchronous TX routine for LPUARTs with a FIFO. Used inside UART_SYNC_TX
// as UART_SYNC_TX_SEND_FIFO_N.
// N is the register number.
#define LPUART_SYNC_TX_SEND_FIFO(N)                          \
  while (len > 0 && ((LPUART##N##_WATER >> 8) & 0x07) < 4) { \
    LPUART##N##_DATA = *(b++);                               \
    len--;                                                   \
  }

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

// Flushes a UART's FIFO. Used inside UART_TX_BREAK.
// N is the register number.
#define UART_TX_FLUSH_FIFO(N)        \
  while (UART##N##_TCFIFO > 0) {     \
    /* Wait for the FIFO to drain */ \
  }

// Transmits a BREAK from a UART.
// Needs to have UART_TX_FLUSH_FIFO_N defined.
// N is the register number.
#define UART_TX_BREAK(N)                     \
  UART_TX_FLUSH_FIFO_##N                     \
                                             \
  while ((UART##N##_S1 & UART_S1_TC) == 0) { \
    /* Wait until we can transmit */         \
  }                                          \
                                             \
  if (breakTime > 0) {                       \
    UART##N##_C3 |= UART_C3_TXINV;           \
    delayMicroseconds(breakTime);            \
    UART##N##_C3 &= ~UART_C3_TXINV;          \
  }                                          \
  delayMicroseconds(mabTime);

// Flushes an LPUART's FIFO. Used inside LPUART_TX_BREAK.
// N is the register number.
#define LPUART_TX_FLUSH_FIFO(N)                   \
  while (((LPUART##N##_WATER >> 8) & 0x07) > 0) { \
    /* Wait for the FIFO to drain */              \
  }

// Transmits a BREAK from an LPUART.
// Needs to have LPUART_TX_FLUSH_FIFO_N defined.
// N is the register number.
#define LPUART_TX_BREAK(N)                           \
  LPUART_TX_FLUSH_FIFO_##N                           \
                                                     \
  while ((LPUART##N##_STAT & LPUART_STAT_TC) == 0) { \
    /* Wait until we can transmit */                 \
  }                                                  \
                                                     \
  if (breakTime > 0) {                               \
    LPUART##N##_CTRL |= LPUART_CTRL_TXINV;           \
    delayMicroseconds(breakTime);                    \
    LPUART##N##_CTRL &= ~LPUART_CTRL_TXINV;          \
  }                                                  \
  delayMicroseconds(mabTime);

#endif  // TEENSYDMX_UART_ROUTINE_DEFINES_H_
