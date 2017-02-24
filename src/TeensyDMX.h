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

// Declare these outside of a namespace
void uart0_error_isr();
void uart1_error_isr();
void uart2_error_isr();

namespace qindesign {
namespace teensydmx {

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

  // Returns whether a new packet was received and is available since the
  // last time this was called. The packet can be retrieved via packet().
  bool packetAvailable();

  // This will return the number of bytes actually read in the last packet,
  // usually 513, but possibly less. This will only return a valid number if
  // packetAvailable() returned true.
  //
  // This method can be used to track buffer underruns, but isn't necessary
  // for usual DMX operation.
  int packetSize() const {
    return packetSize_;
  }

  // Returns the last packet received, if packetAvailable() returned true.
  // The contents should be used "soon", before it is overwritten.
  //
  // This returns up to a 513-byte buffer, with element 0 being the packet's
  // start code, usually 0.
  //
  // Note that packet(), packetAvailable(), and packetSize() are not
  // mutually atomic. See readPacket() for a more complete solution.
  const volatile uint8_t *packet() const {
    return inactiveBuf_;
  }

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
   HardwareSerial &uart_;
   uint8_t oldRWFIFO_;
   bool began_;

   uint8_t buf1_[kMaxDMXPacketSize];
   uint8_t buf2_[kMaxDMXPacketSize];
   uint8_t *activeBuf_;
   volatile uint8_t *inactiveBuf_;
   int bufIndex_;

   volatile unsigned int packetCount_;
   volatile bool packetAvail_;
   volatile bool packetSize_;

   // The current read technique is to fill the buffer after a break is
   // detected, but the break indicates a packet start, not a packet end.
   // Therefore, we're always one behind, and so the first break must not
   // cause a valid packet collection.
   bool first_;

   // Fills the buffer from the UART and then completes the packet from
   // immediately before the break. This reads up to a maximum of
   // kMaxDMXPacketSize bytes and ignores anything after that until
   // the next break.
   //
   // This will be called from an ISR.
   void completePacket();

   // Flushes everything in the UART input buffers. This is used when a
   // framing error is detected but isn't a break (data != 0).
   //
   // This will be called from an ISR.
   void flushInput() const;

   // These error ISR's need to access completePacket().
   friend void ::uart0_error_isr();
   friend void ::uart1_error_isr();
   friend void ::uart2_error_isr();
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // QINDESIGN_TEENSYDMX_H_
