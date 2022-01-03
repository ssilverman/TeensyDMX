// This file is part of the TeensyDMX library.
// (c) 2019-2022 Shawn Silverman

#if defined(__IMXRT1062__) || defined(__IMXRT1052__) || defined(__MK66FX1M0__)

#ifndef TEENSYDMX_LPUARTRECEIVEHANDLER_H_
#define TEENSYDMX_LPUARTRECEIVEHANDLER_H_

// C++ includes
#include <cstdint>

#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
#include <imxrt.h>
using PortType = IMXRT_LPUART_t;
#elif defined(__MK66FX1M0__)
#include <kinetis.h>
using PortType = KINETISK_LPUART_t;
#endif  // Which chip?

#include "ReceiveHandler.h"
#include "TeensyDMX.h"

namespace qindesign {
namespace teensydmx {

class LPUARTReceiveHandler final : public ReceiveHandler {
 public:
  LPUARTReceiveHandler(int serialIndex,
                       Receiver *receiver,
                       PortType *port,
                       IRQ_NUMBER_t irq,
                       void (*irqHandler)())
      : ReceiveHandler(serialIndex, receiver),
        port_(port),
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
        txFIFOSizeSet_(false),
        txFIFOSize_(1),
#endif  // __IMXRT1062__ || __IMXRT1052__
        irq_(irq),
        irqHandler_(irqHandler) {}

  ~LPUARTReceiveHandler() override = default;

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
  PortType *port_;
#if defined(__IMXRT1062__) || defined(__IMXRT1052__)
  bool txFIFOSizeSet_;
  uint32_t txFIFOSize_;
#endif  // __IMXRT1062__ || __IMXRT1052__
  IRQ_NUMBER_t irq_;
  void (*const irqHandler_)();
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_LPUARTRECEIVEHANDLER_H_

#endif  // __IMXRT1062__ || __IMXRT1052__ || __MK66FX1M0__
