// This file is part of the TeensyDMX library.
// (c) 2019-2022 Shawn Silverman

#if defined(__IMXRT1062__) || defined(__IMXRT1052__) || defined(__MK66FX1M0__)

#ifndef TEENSYDMX_LPUARTSENDHANDLER_H_
#define TEENSYDMX_LPUARTSENDHANDLER_H_

// C++ includes
#include <cstdint>

#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
#include <imxrt.h>
using PortType = IMXRT_LPUART_t;
#elif defined(__MK66FX1M0__)
#include <kinetis.h>
using PortType = KINETISK_LPUART_t;
#endif  // Which chip?

#include "SendHandler.h"
#include "TeensyDMX.h"

namespace qindesign {
namespace teensydmx {

class LPUARTSendHandler final : public SendHandler {
 public:
  LPUARTSendHandler(int serialIndex,
                    Sender *sender,
                    PortType *port,
                    IRQ_NUMBER_t irq,
                    void (*irqHandler)())
      : SendHandler(serialIndex, sender),
        port_(port),
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
        fifoSizeSet_(false),
        fifoSize_(1),
#endif  // __IMXRT1062__ || __IMXRT1052__
        irq_(irq),
        irqHandler_(irqHandler),
        slotsSerialParamsSet_(false) {}

  ~LPUARTSendHandler() override = default;

  void start() override;
  void end() const override;
  void setActive() const override;
  void setIRQState(bool flag) const override;
  int priority() const override;
  void irqHandler() const override;

 private:
  // Stored LPUART parameters for quickly setting the baud rate between BREAK
  // and slots. Used for Teensy 3.6 and Teensy 4.
  struct SerialParams final {
    uint32_t baud = 0x0f000004;  // 5-bit OSR is 0x0f and 13-bit SBR is 0x0004
    uint32_t stat = 0;
    uint32_t ctrl = 0;

    void getFrom(PortType *port) {
      baud = port->BAUD;
      stat = port->STAT;
      ctrl = port->CTRL;
    }

    void apply(PortType *port) const {
      port->BAUD = baud;
      port->STAT = stat;
      port->CTRL = ctrl;
    }
  };

  // Set CTRL states
  void setInactive() const;
  void setCompleting() const;

  // Timer handling
  void breakTimerCallback() const;      // When the timer triggers
  void breakTimerPreCallback() const;   // Just before the timer starts
  void interSlotTimerCallback() const;  // When the timer triggers
  void rateTimerCallback() const;       // After the MBB delay

  PortType *port_;
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
  bool fifoSizeSet_;
  uint32_t fifoSize_;
#endif  // __IMXRT1062__ || __IMXRT1052__
  IRQ_NUMBER_t irq_;
  void (*irqHandler_)();

  bool slotsSerialParamsSet_;
  SerialParams breakSerialParams_;
  SerialParams slotsSerialParams_;
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_LPUARTSENDHANDLER_H_

#endif  // __IMXRT1062__ || __IMXRT1052__ || __MK66FX1M0__
