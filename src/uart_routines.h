#ifndef UART_ROUTINES_H_
#define UART_ROUTINES_H_

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

#endif  // UART_ROUTINES_H_
