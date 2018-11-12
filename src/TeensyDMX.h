/*
 * Library for doing DMX on a Teensy. Note that transmit and receive can't
 * be done on the same serial port.
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

// C++ includes
#include <cmath>
#include <cstdint>

// Other includes
#include <Arduino.h>

namespace qindesign {
namespace teensydmx {

void uart0_rx_status_isr();
void uart0_rx_error_isr();
void uart0_tx_status_isr();

void uart1_rx_status_isr();
void uart1_rx_error_isr();
void uart1_tx_status_isr();

void uart2_rx_status_isr();
void uart2_rx_error_isr();
void uart2_tx_status_isr();

#ifdef HAS_KINETISK_UART3
void uart3_rx_status_isr();
void uart3_rx_error_isr();
void uart3_tx_status_isr();
#endif  // HAS_KINETISK_UART3

#ifdef HAS_KINETISK_UART4
void uart4_rx_status_isr();
void uart4_rx_error_isr();
void uart4_tx_status_isr();
#endif  // HAS_KINETISK_UART4

#ifdef HAS_KINETISK_UART5
void uart5_rx_status_isr();
void uart5_rx_error_isr();
void uart5_tx_status_isr();
#endif  // HAS_KINETISK_UART5

#ifdef HAS_KINETISK_LPUART0
void lpuart0_rx_isr();
void lpuart0_tx_isr();
#endif  // HAS_KINETISK_LPUART0

// The maximum size of a DMX packet, including the start code.
constexpr int kMaxDMXPacketSize = 513;

// The minimum size of a DMX packet, including the start code.
// This value is used for senders and is a guideline for how many
// slots will fit in a packet, assuming full-speed transmission.
constexpr int kMinDMXPacketSize = 25;

// TeensyDMX implements either a receiver or transmitter on one of
// hardware serial ports 1-6.
class TeensyDMX {
 public:
  // Sets up the system for receiving or transmitting DMX on the specified
  // serial port.
  virtual void begin() = 0;

  // Tells the system to stop receiving or transmitting DMX. Call this
  // to clean up.
  virtual void end() = 0;

  // Returns the total number of packets received or transmitted since
  // the reciever was started.
  uint32_t packetCount() const { return packetCount_; }

 protected:
  // Creates a new DMX receiver or transmitter using the given hardware UART.
  TeensyDMX(HardwareSerial &uart)
      : uart_(uart),
        began_(false),
        packetCount_(0) {
    serialIndex_ = serialIndex(uart_);
  }

  // TeensyDMX is neither copyable nor movable.
  TeensyDMX(const TeensyDMX&) = delete;
  TeensyDMX& operator=(const TeensyDMX&) = delete;

  virtual ~TeensyDMX() = default;

  // Returns the index given a serial port, or -1 if the serial port is
  // not supported.
  static int serialIndex(HardwareSerial &uart) {
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

  HardwareSerial &uart_;  // TODO: Should this be volatile?
  int serialIndex_;

  // Tracks whether the system has been configured. Subclasses must manage
  // this state.
  bool began_;

  // The number of packets sent or received. Subclasses must manage this.
  volatile uint32_t packetCount_;
};

// ---------------------------------------------------------------------------
//  Receiver
// ---------------------------------------------------------------------------

// A DMX receiver.
class Receiver final : public TeensyDMX {
 public:
  Receiver(HardwareSerial &uart)
      : TeensyDMX(uart),
        buf1_{0},
        buf2_{0},
        activeBuf_(buf1_),
        inactiveBuf_(buf2_),
        activeBufIndex_(0),
        packetSize_(0),
        inPacket_(false),
        lastBreakTime_(0),
        packetTimeoutCount_(0),
        framingErrorCount_(0) {}

  // Destructs Receiver. This calls end().
  ~Receiver() override {
    end();
  }

  // The maximum allowed packet time for receivers, either BREAK plus data,
  // or BREAK to BREAK, in milliseconds.
  static constexpr uint32_t kMaxDMXPacketTime = 1250;

  void begin() override;

  void end() override;

  // Reads all or part of a packet into buf. This does nothing if len
  // is negative or zero, or if startChannel is negative or beyond
  // kMaxDMXPacketSize, and only reads up to the end of the packet
  // if startChannel + len would go past the end.
  //
  // This will return the number of bytes actually read into buf, -1 if there
  // is no packet available since the last call to this function, or zero if
  // len is negative or zero, or if the requested data is outside the range
  // of the recieved packet. Differentiating between -1 and zero allows the
  // caller to determine whether there was no packet received or a packet
  // was received and did not contain the requested data.
  //
  // The values starting at startChannel will be stored starting at index
  // zero in buf. buf must have a size of at least len bytes.
  int readPacket(uint8_t *buf, int startChannel, int len);

  // Gets the value for one channel. The start code can be read at
  // channel zero.
  //
  // If the channel is out of range then this will return zero.
  uint8_t get(int channel) const {
    if (channel <= 0 || kMaxDMXPacketSize <= channel) {
      return 0;
    }
    return inactiveBuf_[channel];
  }

  // Returns the timestamp of the last received packet. Under the covers,
  // millis() is called when a packet is received. Note that this may not
  // indicate freshness of the channels you're interested in because they
  // may not have been a part of the last packet received. i.e. the last
  // packet received may have been smaller than required.
  //
  // Use of this function is discouraged in favor of making a note of the
  // time inside the code that checks for the value returned from
  // readPacket() being at least equal to the number of channels requested.
  // Use is only suggested in very simple use cases where a timestamp is
  // needed for *any* packet containing data, not necessarily desired data.
  //
  // Don't use this function to detect timeouts for data on specific channels.
  // For example, unplugging a cable might result in a valid packet, but not
  // containing the channels you need. Using this value to detect the last
  // valid data received would give a value that's later than the true value.
  uint32_t lastPacketTimestamp() const {
    return packetTimestamp_;
  }

  // Returns the total number of packets received or transmitted since
  // the reciever was started.
  uint32_t packetTimeoutCount() const {
    return packetTimeoutCount_;
  }

  // Returns the total number of framing errors encountered.
  uint32_t framingErrorCount() const {
    return framingErrorCount_;
  }

 private:
  // Disables all the UART IRQ's so that variables can be accessed concurrently.
  void disableIRQs();

  // Enables all the UART IRQ's.
  void enableIRQs();

  // Makes a new packet available.
  // This will be called from an ISR.
  void completePacket();

  // A break has just been received.
  // This will be called from an ISR.
  void receiveBreak();

  // Receives a byte.
  // This will be called from an ISR.
  void receiveByte(uint8_t b);

  volatile uint8_t buf1_[kMaxDMXPacketSize];
  volatile uint8_t buf2_[kMaxDMXPacketSize];
  volatile uint8_t *activeBuf_;
  // Read-only shared memory buffer, make const volatile
  // https://embeddedgurus.com/barr-code/2012/01/combining-cs-volatile-and-const-keywords/
  const volatile uint8_t *inactiveBuf_;
  volatile int activeBufIndex_;

  // The size of the last received packet.
  volatile int packetSize_;

  // The timestamp of the last received packet.
  volatile uint32_t packetTimestamp_;

  // Indicates that we are inside a packet; a BREAK was received.
  volatile bool inPacket_;

  // For timing
  volatile uint32_t lastBreakTime_;

  // Counts
  volatile uint32_t packetTimeoutCount_;
  volatile uint32_t framingErrorCount_;

  // These error ISR's need to access private functions
  friend void uart0_rx_status_isr();
  friend void uart0_rx_error_isr();
  friend void uart1_rx_status_isr();
  friend void uart1_rx_error_isr();
  friend void uart2_rx_status_isr();
  friend void uart2_rx_error_isr();
#ifdef HAS_KINETISK_UART3
  friend void uart3_rx_status_isr();
  friend void uart3_rx_error_isr();
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
  friend void uart4_rx_status_isr();
  friend void uart4_rx_error_isr();
#endif  // HAS_KINETISK_UART4
#ifdef HAS_KINETISK_UART5
  friend void uart5_rx_status_isr();
  friend void uart5_rx_error_isr();
#endif  // HAS_KINETISK_UART5
#ifdef HAS_KINETISK_LPUART0
friend void lpuart0_rx_isr();
#endif  // HAS_KINETISK_LPUART0
};

// ---------------------------------------------------------------------------
//  Sender
// ---------------------------------------------------------------------------

// A DMX transmitter.
class Sender final : public TeensyDMX {
 public:
  Sender(HardwareSerial &uart)
      : TeensyDMX(uart),
        state_(XmitStates::kIdle),
        outputBuf_{0},
        outputBufIndex_(0),
        packetSize_(kMaxDMXPacketSize),
        refreshRate_(INFINITY),
        breakToBreakTime_(0),
        markBeforeBreakTime_(0) {}

  // Destructs Sender. This calls end().
  ~Sender() override {
    end();
  }

  void begin() override;

  void end() override;

  // Sets the transmit packet size, in number of channels plus the start code.
  // This does nothing if the size is outside the range 25-513. These limits
  // are contained in kMinDMXPacketSize and kMaxDMXPacketSize, respectively.
  //
  // The default is 513.
  //
  // The reason for the lower bound is so that, under conditions where minimum
  // timings are used, the total packet time does not fall below 1204us, per
  // the ANSI E1.11 DMX specification.
  void setPacketSize(int size) {
    if (kMinDMXPacketSize <= size && size <= kMaxDMXPacketSize) {
      packetSize_ = size;
    }
  }

  // Sets a channel's value. Channel zero represents the start code.
  // The start code should really be zero, but it can be changed here.
  //
  // If the channel is out of range then the call is ignored.
  void set(int channel, uint8_t value) {
    if (0 <= channel && channel < kMaxDMXPacketSize) {
      outputBuf_[channel] = value;
    }
  }

  // Sets the values for a range of channels.
  //
  // This does nothing if any part of the channel range is not in the
  // range 0-512. This limit is equal to kDMXMaxPacketSize-1.
  void set(int startChannel, const uint8_t *values, int len);

  // Sets the packet refresh rate. Negative and NaN values are ignored.
  //
  // If the rate is too high then this will simply transmit as fast
  // as possible. Transmitting as fast as possible is also the default.
  void setRefreshRate(float rate);

 private:
   // State that tracks what to transmit and when.
   enum XmitStates {
     kBreak,  // Need to transmit a break
     kData,   // Need to transmit data
     kIdle,   // The end of data for one packet has been reached
   };

  // The minimum allowed packet time for senders, either BREAK plus data,
  // or BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMinDMXPacketTime = 1204;

  // Completes a sent packet. This increments the packet count, resets the
  // output buffer index, and sets the state to Idle.
  //
  // This will be called from an ISR.
  void completePacket();

  // Keeps track of what we're transmitting.
  volatile XmitStates state_;

  volatile uint8_t outputBuf_[kMaxDMXPacketSize];
  int outputBufIndex_;

  // The size of the packet to be sent.
  volatile int packetSize_;

  // The packet refresh rate, in Hz.
  float refreshRate_;

  // The BREAK-to-BREAK timing, matching the refresh rate. This is
  // specified in microseconds.
  uint32_t breakToBreakTime_;

  // The time to wait before sending the next BREAK. This changes for
  // each packet.
  uint32_t markBeforeBreakTime_;

  // Keeps track of when we should send the next break.
  // In the Break and Data states, it means time since the break.
  // In the Idle state, it means time since the end-of-packet.
  elapsedMicros timeSince_;

  // These error ISR's need to access private functions
  friend void uart0_tx_status_isr();
  friend void uart1_tx_status_isr();
  friend void uart2_tx_status_isr();
#ifdef HAS_KINETISK_UART3
  friend void uart3_tx_status_isr();
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
  friend void uart4_tx_status_isr();
#endif  // HAS_KINETISK_UART4
#ifdef HAS_KINETISK_UART5
  friend void uart5_tx_status_isr();
#endif  // HAS_KINETISK_UART5
#ifdef HAS_KINETISK_LPUART0
  friend void lpuart0_tx_isr();
#endif  // HAS_KINETISK_LPUART0
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // QINDESIGN_TEENSYDMX_H_
