#include "TeensyDMX.h"

// Other includes
#include <HardwareSerial.h>

namespace qindesign {
namespace teensydmx {

// Returns the index given a serial port, or -1 if the serial port is
// not supported.
int serialIndex(HardwareSerial &uart) {
  if (&uart == &Serial1) {
    return 0;
  }
  if (&uart == &Serial2) {
    return 1;
  }
  if (&uart == &Serial3) {
    return 2;
  }
#ifdef HAS_KINETISK_UART3
  if (&uart == &Serial4) {
    return 3;
  }
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
  if (&uart == &Serial5) {
    return 4;
  }
#endif  // HAS_KINETISK_UART4
#if defined(HAS_KINETISK_UART5)
  if (&uart == &Serial6) {
    return 5;
  }
#elif defined(HAS_KINETISK_LPUART0)
  if (&uart == &Serial6) {
    return 5;
  }
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0
  return -1;
}

TeensyDMX::TeensyDMX(HardwareSerial &uart)
    : uart_(uart),
      began_(false),
      packetCount_(0) {
  serialIndex_ = serialIndex(uart_);
}

}  // namespace teensydmx
}  // namespace qindesign
