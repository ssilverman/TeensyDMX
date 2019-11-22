// This file is part of the TeensyDMX library.
// (c) 2017-2019 Shawn Silverman

#include "TeensyDMX.h"

// Other includes
#include <HardwareSerial.h>

namespace qindesign {
namespace teensydmx {

// Returns the index given a serial port, or -1 if the serial port is
// not supported.
int serialIndex(const HardwareSerial &uart) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
  if (&uart == &Serial1) {
    return 0;
  }
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
  if (&uart == &Serial2) {
    return 1;
  }
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
  if (&uart == &Serial3) {
    return 2;
  }
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
  if (&uart == &Serial4) {
    return 3;
  }
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
  if (&uart == &Serial5) {
    return 4;
  }
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5) || defined(HAS_KINETISK_LPUART0)
  if (&uart == &Serial6) {
    return 5;
  }
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  return -1;
}

TeensyDMX::TeensyDMX(HardwareSerial &uart)
    : uart_(uart),
      serialIndex_(serialIndex(uart_)),
      packetCount_(0) {}

}  // namespace teensydmx
}  // namespace qindesign
