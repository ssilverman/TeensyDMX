// This file is part of the TeensyDMX library.
// (c) 2019-2022 Shawn Silverman

#if defined(__MK20DX128__) || defined(__MK20DX256__) || \
    defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

#ifndef TEENSYDMX_UARTRECEIVEHANDLER_H_
#define TEENSYDMX_UARTRECEIVEHANDLER_H_

// C++ includes
#include <cstdint>

#include <kinetis.h>

#include "ReceiveHandler.h"
#include "TeensyDMX.h"

namespace qindesign {
namespace teensydmx {

class UARTReceiveHandler final : public ReceiveHandler {
 public:
  UARTReceiveHandler(int serialIndex,
                     Receiver *receiver,
                     KINETISK_UART_t *port,
                     IRQ_NUMBER_t irq,
#if defined(KINETISK)
                     IRQ_NUMBER_t errorIRQ,
#endif  // KINETISK
                     void (*irqHandler)())
      : ReceiveHandler(serialIndex, receiver),
        port_(port),
#if defined(KINETISK)
        fifoSizesSet_(false),
        rxFIFOSize_(1),
        txFIFOSize_(1),
#endif  // KINETISK
        irq_(irq),
#if defined(KINETISK)
        errorIRQ_(errorIRQ),
#endif  // KINETISK
        irqHandler_(irqHandler) {}

  ~UARTReceiveHandler() override = default;

  void start() override;
  void end() const override;
  void setTXEnabled(bool flag) const override;
  void setILT(bool flag) const override;
  void setIRQState(bool flag) const override;
  int priority() const override;
  void irqHandler() const override;
  void txData(const uint8_t *b, int len) const override;
  void txBreak(uint32_t breakTime, uint32_t mabTime) const override;

 private:
  KINETISK_UART_t *port_;
#if defined(KINETISK)
  bool fifoSizesSet_;
  uint8_t rxFIFOSize_;
  uint8_t txFIFOSize_;
#endif  // KINETISK
  IRQ_NUMBER_t irq_;
#if defined(KINETISK)
  IRQ_NUMBER_t errorIRQ_;
#endif  // KINETISK
  void (*irqHandler_)();
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_UARTRECEIVEHANDLER_H_

#endif  // __MK20DX128__ || __MK20DX256__ || __MKL26Z64__ || __MK64FX512__ ||
        // __MK66FX1M0__
