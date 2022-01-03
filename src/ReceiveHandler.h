// ReceiveHandler.h defines the base class for the code that handles RX things.
// This file is part of the TeensyDMX library.
// (c) 2019-2022 Shawn Silverman

#ifndef TEENSYDMX_RECEIVEHANDLER_H_
#define TEENSYDMX_RECEIVEHANDLER_H_

// C++ includes
#include <cstdint>

namespace qindesign {
namespace teensydmx {

class Receiver;

// Defines this interface.
class ReceiveHandler {
 public:
  virtual ~ReceiveHandler() = default;

  // Starts the UART. This calls `begin` with the slots baud rate and format,
  // and then activates the interrupts, sets the Idle Line Type Select to "Idle
  // starts after start bit", and attaches the interrupt routines.
  virtual void start() = 0;

  // Stops the serial port. This also disables any additional
  // enabled interrupts.
  virtual void end() const = 0;

  // Enables or disables TX.
  virtual void setTXEnabled(bool flag) const = 0;

  // Enables or disables the Idle Line Type Select (ILT) flag. `false` for "Idle
  // starts after start bit" and `true` for "Idle starts after stop bit".
  virtual void setILT(bool flag) const = 0;

  // Enables or disables the UART IRQ(s).
  virtual void setIRQState(bool flag) const = 0;

  // Returns the priority of the internal IRQ.
  virtual int priority() const = 0;

  // Handles interrupts.
  virtual void irqHandler() const = 0;

  // Sends synchronous data.
  virtual void txData(const uint8_t *b, int len) const = 0;

  // Sends a synchronous BREAK and MAB.
  virtual void txBreak(uint32_t breakTime, uint32_t mabTime) const = 0;

 protected:
  ReceiveHandler(int serialIndex, Receiver *receiver)
      : serialIndex_(serialIndex),
        receiver_(receiver) {}

  const int serialIndex_;
  Receiver *receiver_;
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_RECEIVEHANDLER_H_
