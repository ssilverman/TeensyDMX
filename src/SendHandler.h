// SendHandler.h defines the base class for the code that handles TX things.
// This file is part of the TeensyDMX library.
// (c) 2019-2021 Shawn Silverman

#ifndef TEENSYDMX_SENDHANDLER_H_
#define TEENSYDMX_SENDHANDLER_H_

#include <cstdint>

namespace qindesign {
namespace teensydmx {

class Sender;

// Defines this interface.
class SendHandler {
 public:
  virtual ~SendHandler() = default;

  // Indicates that the BREAK/MAB serial parameters have changed.
  void breakSerialParamsChanged() {
    breakSerialParamsChanged_ = true;
  }

  // Starts the UART with the slots baud rate. If the baud rate parameters have
  // not been set then this sets the baud rate to the BREAK baud rate and then
  // the slots baud rate, and gleans both sets of register parameters.
  //
  // After that, this attaches the ISRs but does not put the UART into
  // "ACTIVE" mode.
  virtual void start() = 0;

  // Stops the UART.
  virtual void end() const = 0;

  // Enables or disables the UART IRQ(s).
  virtual void setIRQState(bool flag) const = 0;

  // Returns the priority of the internal IRQ.
  virtual int priority() const = 0;

  // Puts the UART into "ACTIVE" mode. The UART should have already been started
  // before calling this function, otherwise the chip may hang.
  virtual void setActive() const = 0;

  // Handles interrupts.
  virtual void irqHandler() const = 0;

 protected:
  SendHandler(int serialIndex, Sender *sender)
      : serialIndex_(serialIndex),
        sender_(sender),
        breakSerialParamsChanged_(true) {}

  const int serialIndex_;
  Sender *sender_;

  bool breakSerialParamsChanged_;
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_SENDHANDLER_H_
