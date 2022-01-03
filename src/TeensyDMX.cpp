// This file is part of the TeensyDMX library.
// (c) 2017-2022 Shawn Silverman

#include "TeensyDMX.h"

namespace qindesign {
namespace teensydmx {

extern const uint32_t kSlotsBaud   = 250000;                // 4us
extern const uint32_t kSlotsFormat = SERIAL_8N2;            // 9:2
extern const uint32_t kBitTime     = 1000000 / kSlotsBaud;  // In microseconds
extern const uint32_t kCharTime    = 11 * kBitTime;         // In microseconds

// Returns the index given a serial port, or -1 if the serial port is
// not supported.
constexpr int serialIndex(const HardwareSerial &uart) {
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0) || \
    defined(IMXRT_LPUART6)
  if (&uart == &Serial1) {
    return 0;
  }
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0 || IMXRT_LPUART6

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1) || \
    defined(IMXRT_LPUART4)
  if (&uart == &Serial2) {
    return 1;
  }
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1 || IMXRT_LPUART4

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2) || \
    defined(IMXRT_LPUART2)
  if (&uart == &Serial3) {
    return 2;
  }
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2 || IMXRT_LPUART2

#if defined(HAS_KINETISK_UART3) || defined(IMXRT_LPUART3)
  if (&uart == &Serial4) {
    return 3;
  }
#endif  // HAS_KINETISK_UART3 || IMXRT_LPUART3

#if defined(HAS_KINETISK_UART4) || defined(IMXRT_LPUART8)
  if (&uart == &Serial5) {
    return 4;
  }
#endif  // HAS_KINETISK_UART4 || IMXRT_LPUART8

#if defined(HAS_KINETISK_UART5) || defined(HAS_KINETISK_LPUART0) || \
    defined(IMXRT_LPUART1)
  if (&uart == &Serial6) {
    return 5;
  }
#endif  // HAS_KINETISK_UART5 || HAS_KINETISK_LPUART0 || IMXRT_LPUART1

#if defined(IMXRT_LPUART7)
  if (&uart == &Serial7) {
    return 6;
  }
#endif  // IMXRT_LPUART7

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))
  if (&uart == &Serial8) {
    return 7;
  }
#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)

  return -1;
}

TeensyDMX::TeensyDMX(HardwareSerial &uart)
    : uart_(uart),
      serialIndex_(serialIndex(uart_)),
      packetCount_(0) {}

}  // namespace teensydmx
}  // namespace qindesign
