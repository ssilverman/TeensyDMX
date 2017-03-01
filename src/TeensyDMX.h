/*
 * Library for doing DMX on a Teensy.
 *
 * (c) 2017 Shawn Silverman
 */

/*
  Links:
  https://github.com/jimparis/DmxReceiver
  https://forum.pjrc.com/threads/19662-Arduinoesque-overriding-of-core-functionality?highlight=teensydmx
  https://www.holidaycoro.com/v/vspfiles/downloads/UnderstandingDMX.pdf
  https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf
*/

#ifndef QINDESIGN_TEENSYDMX_H_
#define QINDESIGN_TEENSYDMX_H_

#include <Arduino.h>
#include <inttypes.h>

namespace qindesign {
namespace teensydmx {

void uart0_status_isr();
void uart0_error_isr();
void uart1_status_isr();
void uart1_error_isr();
void uart2_status_isr();
void uart2_error_isr();

// The current incarnation of this class implements only a DMX receiver
// on hardware serial ports 1-3. For whichever serial port is being used,
// it is necessary to #define the receive buffer size to be 513 bytes.
//
// For example, to define the receive buffer size for Serial1:
//   #define SERIAL1_RX_BUFFER SIZE 513
class TeensyDMX {
 public:
  // The maximum size of a DMX packet. See packet().
  //
  // This size also includes the start code.
  static const int kMaxDMXPacketSize = 513;

  TeensyDMX(HardwareSerial &uart);

  // TeensyDMX is neither copyable nor movable.
  TeensyDMX(const TeensyDMX&) = delete;
  TeensyDMX& operator=(const TeensyDMX&) = delete;

  // Set up the system for receiving on the specified serial port.
  void begin();

  // Tells the system to stop receiving DMX. Call this to clean up.
  void end();

  // Returns the total number of packets received since the reciever
  // was started.
  unsigned int packetCount() const {
    return packetCount_;
  }

  // Reads a packet into buf. The number of bytes can be up to
  // kMaxDMXPacketSize. This will return -1 if there is no packet
  // available.
  //
  // This will reset the state of a packet being available. In other words,
  // this has the same effect as calling packetAvailable(): the next time
  // this is called, this will return -1.
  int readPacket(uint8_t *buf);

 private:
   // Fills the buffer from the UART and then completes the packet from
   // immediately before the break. This reads up to a maximum of
   // kMaxDMXPacketSize bytes and ignores anything after that until
   // the next break.
   //
   // This will be called from an ISR.
   void completePacket();

   // Resets the packet on a framing error.
   // This will be called from an ISR.
   void resetPacket();

   // Receives a byte.
   // This will be called from an ISR.
   void receiveByte(uint8_t b);

   // These error ISR's need to access private functions
   friend void uart0_status_isr();
   friend void uart0_error_isr();
   friend void uart1_status_isr();
   friend void uart1_error_isr();
   friend void uart2_status_isr();
   friend void uart2_error_isr();

   HardwareSerial &uart_;
   bool began_;

   // For disabling/enabling interrupts, for concurrency
   int irqNumber_;

   uint8_t buf1_[kMaxDMXPacketSize];
   uint8_t buf2_[kMaxDMXPacketSize];
   uint8_t *activeBuf_;
   volatile uint8_t *inactiveBuf_;

   int activeBufIndex_;
   volatile unsigned int packetCount_;
   volatile int packetSize_;

   // The current read technique is to fill the buffer after a break is
   // detected, but the break indicates a packet start, not a packet end.
   // Therefore, we're always one behind, and so the first break must not
   // cause a valid packet collection.
   bool first_;
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // QINDESIGN_TEENSYDMX_H_
