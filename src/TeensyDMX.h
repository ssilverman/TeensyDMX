/*
 * Library for doing DMX on a Teensy. Note that transmit and receive can't
 * be done on the same serial port.
 *
 * (c) 2017-2019 Shawn Silverman
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
#include <cstdint>

// Other includes
#include <Arduino.h>

// Project includes
#include "Responder.h"

namespace qindesign {
namespace teensydmx {

void uart0_rx_isr();
void uart0_tx_isr();

void uart1_rx_isr();
void uart1_tx_isr();

void uart2_rx_isr();
void uart2_tx_isr();

#ifdef HAS_KINETISK_UART3
void uart3_rx_isr();
void uart3_tx_isr();
#endif  // HAS_KINETISK_UART3

#ifdef HAS_KINETISK_UART4
void uart4_rx_isr();
void uart4_tx_isr();
#endif  // HAS_KINETISK_UART4

#ifdef HAS_KINETISK_UART5
void uart5_rx_isr();
void uart5_tx_isr();
#endif  // HAS_KINETISK_UART5

#ifdef HAS_KINETISK_LPUART0
void lpuart0_rx_isr();
void lpuart0_tx_isr();
#endif  // HAS_KINETISK_LPUART0

// The maximum size of a DMX packet, including the start code.
constexpr int kMaxDMXPacketSize = 513;

// The minimum size of a DMX packet, including the start code. This value is
// used for senders and is a guideline for how many slots will fit in a packet,
// assuming full-speed transmission.
constexpr int kMinDMXPacketSize = 25;

// TeensyDMX implements either a receiver or transmitter on one of hardware
// serial ports 1-6.
class TeensyDMX {
 public:
  // Sets up the system for receiving or transmitting DMX on the specified
  // serial port.
  virtual void begin() = 0;

  // Tells the system to stop receiving or transmitting DMX. Call this to
  // clean up.
  virtual void end() = 0;

  // Returns the total number of packets received or transmitted since the
  // receiver was started.
  uint32_t packetCount() const {
    return packetCount_;
  }

 protected:
  // Creates a new DMX receiver or transmitter using the given hardware UART.
  TeensyDMX(HardwareSerial &uart);

  // TeensyDMX is neither copyable nor movable.
  TeensyDMX(const TeensyDMX &) = delete;
  TeensyDMX& operator=(const TeensyDMX &) = delete;

  virtual ~TeensyDMX() = default;

  HardwareSerial &uart_;
  int serialIndex_;

  // Tracks whether the system has been configured. Subclasses must manage
  // this state.
  volatile bool began_;

  // The number of packets sent or received. Subclasses must manage this.
  volatile uint32_t packetCount_;
};

// ---------------------------------------------------------------------------
//  Receiver
// ---------------------------------------------------------------------------

// A DMX receiver. This receives packets asynchronously.
class Receiver final : public TeensyDMX {
 public:
  // Creates a new receiver and uses the given UART for communication.
  Receiver(HardwareSerial &uart);

  // Destructs Receiver. This calls end().
  ~Receiver() override;

  // Starts up the serial port.
  //
  // Call setSetTXNotRXFunc() to set an appropriate pin toggle function before
  // calling begin(). If one is set, this will call it to enable receive.
  void begin() override;

  void end() override;

  // Reads all or part of a packet into buf. This returns zero if len is
  // negative or zero, or if startChannel is negative or beyond
  // kMaxDMXPacketSize. This only reads up to the end of the packet if
  // startChannel+len would go past the end.
  //
  // This will return the number of bytes actually read into buf, -1 if there is
  // no packet available since the last call to this function, or zero if len is
  // negative or zero, or if the requested data is outside the range of the
  // received packet. Differentiating between -1 and zero allows the caller to
  // determine whether there was no packet received or a packet was received and
  // did not contain the requested data.
  //
  // The values starting at startChannel will be stored starting at index zero
  // in buf. buf must have a size of at least len bytes.
  int readPacket(uint8_t *buf, int startChannel, int len);

  // Gets the value for one channel. The start code can be read at channel zero.
  //
  // If the channel is out of range then this will return zero. This is
  // equivalent to calling readPacket() with a buffer size of 1, and returning
  // zero for unavailable data or an out of range request.
  uint8_t get(int channel) const;

  // Returns the timestamp of the last received packet. Under the covers,
  // millis() is called when a packet is received. Note that this may not
  // indicate freshness of the channels you're interested in because they may
  // not have been a part of the last packet received. i.e. the last packet
  // received may have been smaller than required.
  //
  // Use of this function is discouraged in favor of making a note of the time
  // inside the code that checks for the value returned from readPacket() being
  // at least equal to the number of channels requested. Use is only suggested
  // in very simple use cases where a timestamp is needed for *any* packet
  // containing data, not necessarily desired data.
  //
  // Don't use this function to detect timeouts for data on specific channels.
  // For example, unplugging a cable might result in a valid packet, but not
  // containing the channels you need. Using this value to detect the last valid
  // data received would give a value that's later than the true value.
  uint32_t lastPacketTimestamp() const {
    return packetTimestamp_;
  }

  // Sets the responder for the supplied start code. This holds on to the
  // pointer, so callers should take care to not free the object before this
  // Receiver is freed or the responder for the start code is set to nullptr.
  //
  // This will replace any responder having the same start code and returns
  // the replaced pointer. Note that this will return nullptr if no responder
  // was replaced.
  //
  // Setting the responder for a start code to nullptr will remove any
  // previously-set responder for that start code.
  Responder *setResponder(uint8_t startCode, Responder *r);

  // Sets the setTXNotRX implementation function. This should be called before
  // calling begin().
  void setSetTXNotRXFunc(void (*f)(bool flag)) {
    setTXNotRXFunc_ = f;
  }

  // Returns whether this is considered to be connected to a DMX transmitter. A
  // connection is considered to have been broken if a timeout was detected or a
  // BREAK plus Mark after Break (MAB) was too short.
  bool connected() const {
    return connected_;
  }

  // Sets the function to call when the connection state changes. This can be
  // used instead of polling connected(). The function takes one argument, a
  // pointer to this Receiver instance.
  //
  // The function is called when the same conditions checked by connected()
  // occur. It is called from an ISR.
  void onConnectChange(void (*f)(Receiver *r)) {
    connectChangeFunc_ = f;
  }

  // Returns the total number of packets received or transmitted since the
  // receiver was started.
  uint32_t packetTimeoutCount() const {
    return packetTimeoutCount_;
  }

  // Returns the total number of framing errors encountered. This includes
  // BREAKs that are too short.
  uint32_t framingErrorCount() const {
    return framingErrorCount_;
  }

  // Returns the total number of packets that were too short.
  uint32_t shortPacketCount() const {
    return shortPacketCount_;
  }

 private:
  // State that tracks where we are in the receive process.
  enum class RecvStates {
    kBreak,  // Break
    kMAB,    // Mark after break
    kData,   // Packet data
    kIdle,   // The end of data for one packet has been reached
  };

  // The maximum allowed packet time for receivers, either BREAK plus data,
  // or BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMaxDMXPacketTime = 1250000;

  // The minimum allowed packet time for receivers, BREAK to BREAK,
  // in microseconds.
  static constexpr uint32_t kMinDMXPacketTime = 1196;

  // The maximum allowed IDLE and Mark Before Break (MBB) time,
  // in microseconds, exclusive.
  static constexpr uint32_t kMaxDMXIdleTime = 1000000;

  // Disables all the UART IRQs so that variables can be accessed concurrently.
  void disableIRQs() const;

  // Enables all the UART IRQs.
  void enableIRQs() const;

  // Called when the connection state changes.
  void setConnected(bool flag);

  // Makes a new packet available and resets state.
  // This is called from an ISR.
  void completePacket();

  // Look for potential packet timeouts.
  // This is called from an ISR.
  void checkPacketTimeout();

  // A potential break has just been received.
  // This is called from an ISR.
  void receivePotentialBreak();

  // An invalid start-of-break was received. There were non-zero bytes in the
  // framing error.
  // This is called from an ISR.
  void receiveBadBreak();

  // Receives a byte.
  // This is called from an ISR.
  void receiveByte(uint8_t b);

  // Sets whether to enable or disable TX or RX through some external means.
  // This is needed when responding to a received message and transmission
  // needs to occur. This is also called at the end of begin() with 'false'
  // to enable RX.
  void setTXNotRX(bool flag) {
    if (setTXNotRXFunc_ == nullptr) {
      return;
    }
    setTXNotRXFunc_(flag);
  }

  // Keeps track of what we're receiving.
  volatile RecvStates state_;

  // The framing-error start time, in microseconds. This needs to be accessed
  // from the same interrupt that triggered the framing error so that there's
  // a guarantee that it doesn't get changed.
  //
  // This is separate from breakStartTime_ because the measurement is done right
  // when the framing error is detected, and before any logic that might consume
  // some time.
  uint32_t feStartTime_;

  uint8_t buf1_[kMaxDMXPacketSize];
  uint8_t buf2_[kMaxDMXPacketSize];
  uint8_t *activeBuf_;
  // Read-only shared memory buffer, make const volatile
  // https://embeddedgurus.com/barr-code/2012/01/combining-cs-volatile-and-const-keywords/
  const uint8_t *volatile inactiveBuf_;
  int activeBufIndex_;

  // The size of the last received packet.
  volatile int packetSize_;

  // The timestamp of the last received packet, in milliseconds.
  volatile uint32_t packetTimestamp_;

  // Current and last BREAK start times, in microseconds. The last start time is
  // zero if we can consider that there's been no prior packet, and the current
  // start time isn't set until it's confirmed that there's been a valid BREAK.
  uint32_t lastBreakStartTime_;
  uint32_t breakStartTime_;

  // Last time a slot ended, in microseconds.
  uint32_t lastSlotEndTime_;

  // Indicates whether we are connected to a DMX transmitter. Disconnection is
  // considered to have occurred when a timeout, framing error, or short packet
  // is detected.
  volatile bool connected_;

  // This is called when the connection state changes.
  void (*volatile connectChangeFunc_)(Receiver *r);

  // Counts
  volatile uint32_t packetTimeoutCount_;
  volatile uint32_t framingErrorCount_;
  volatile uint32_t shortPacketCount_;

  // Responders state
  Responder *volatile *volatile responders_;
  int responderCount_;
  uint8_t *volatile responderOutBuf_;
  int responderOutBufLen_;

  // Function for enabling/disabling RX and TX.
  void (*volatile setTXNotRXFunc_)(bool flag);

  // Transmit function for the current UART.
  void (*txFunc_)(const uint8_t *b, int len);

  // Transmit BREAK function for the current UART. The MAB (mark after break)
  // time is spcified because different UARTs send BREAKs differently.
  void (*txBreakFunc_)(int count, uint32_t mabTime);

  // These error ISRs need to access private functions
  friend void uart0_rx_isr();
  friend void uart1_rx_isr();
  friend void uart2_rx_isr();
#ifdef HAS_KINETISK_UART3
  friend void uart3_rx_isr();
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
  friend void uart4_rx_isr();
#endif  // HAS_KINETISK_UART4
#ifdef HAS_KINETISK_UART5
  friend void uart5_rx_isr();
#endif  // HAS_KINETISK_UART5
#ifdef HAS_KINETISK_LPUART0
  friend void lpuart0_rx_isr();
#endif  // HAS_KINETISK_LPUART0
};

// ---------------------------------------------------------------------------
//  Sender
// ---------------------------------------------------------------------------

// A DMX transmitter. This sends packets asynchronously.
class Sender final : public TeensyDMX {
 public:
  Sender(HardwareSerial &uart);

  // Destructs Sender. This calls end().
  ~Sender() override;

  // Starts up the serial port.
  void begin() override;

  // Ends sending. Note that this does not wait for the current packet to
  // finish. To complete the current packet, pause and then check if
  // transmission is still active. See pause() and isTransmitting().
  void end() override;

  // Sets the transmit packet size, in number of channels plus the start code.
  // This does nothing if the size is greater than 513 or negative.
  //
  // When the maximum refresh rate is used, the packet size should be >= 25 so
  // that the total packet time does not fall below 1204us, per the ANSI E1.11
  // DMX specification. However, smaller packets can be sent if the refresh rate
  // is decreased.
  //
  // These limits are contained in kMaxDMXPacketSize and kMinDMXPacketSize.
  //
  // For example, if the packet size is set to 25, then the channels can range
  // from 0 to 24, inclusive, with channel zero containing the start code and
  // slots 1-24 containing the remainder of the packet data.
  //
  // The default is 513.
  void setPacketSize(int size) {
    if (0 <= size && size <= kMaxDMXPacketSize) {
      packetSize_ = size;
    }
  }

  // Returns the current packet size.
  int packetSize() const {
    return packetSize_;
  }

  // Sets a channel's value. Channel zero represents the start code. The start
  // code should really be zero, but it can be changed here. This also affects
  // the packet currently being transmitted.
  //
  // Values set here are 'sticky'. In other words, the same values are
  // transmitted until changed. To set a value, this only needs to be
  // called once.
  //
  // If the channel is not in the range 0-512 then the call is ignored. Note
  // that it is possible to set channels outside the range of the packet size,
  // but these values will not be sent.
  //
  // For example, if the packet size is 25 and the channel is anywhere in the
  // range 25-512, then the value will be set internally but will not be
  // transmitted until the packet size changes via setPacketSize().
  //
  // After pausing with pause(), it's not necessary to wait until transmission
  // is finished before setting channel values.
  void set(int channel, uint8_t value);

  // Sets the values for a range of channels. This also affects the packet
  // currently being transmitted.
  //
  // Values set here are 'sticky'. In other words, the same values are
  // transmitted until changed. To set some values, this only needs to be
  // called once.
  //
  // This does nothing if any part of the channel range is not in the range
  // 0-512. This limit is equal to kDMXMaxPacketSize-1.
  //
  // See the other 'set' function for more information about setting
  // values outside the range of the current packet size (if the size
  // is less than 513).
  //
  // After pausing with pause(), it's not necessary to wait until transmission
  // is finished before setting channel values.
  void set(int startChannel, const uint8_t *values, int len);

  // Sets the packet refresh rate. Negative and NaN values are ignored. The
  // default is INFINITY, indicating "as fast as possible".
  //
  // If the rate is too high then this will simply transmit as fast as possible.
  // Transmitting as fast as possible is also the default.
  //
  // If the rate is zero then no packets will be sent. However, the serial
  // transmitter will still be enabled. A rate of zero is not equivalent to
  // calling end().
  //
  // If the new rate is non-zero and the former rate is zero then this will call
  // end() and then begin().
  //
  // For rates slower than the maximum, this uses an IntervalTimer internally.
  void setRefreshRate(float rate);

  // Returns the packet refresh rate. The default is INFINITY, indicating
  // "as fast as possible".
  float refreshRate() const {
    return refreshRate_;
  }

  // Pauses the ansynchronous packet sending. This allows information to be
  // inserted at a specific point. This pauses after finishing transmission of
  // any current packet.
  //
  // An example where this is useful is for System Information Packets (SIP),
  // where checksum data needs to be applied to the preceding packet.
  //
  // Note that the current packet does not need to complete before setting
  // channel data with one of the 'set' functions. Use them freely after
  // calling pause().
  //
  // Also note that this does not change the number of resumed packets
  // remaining.
  void pause() {
    paused_ = true;
  }

  // Returns whether we are currently paused. This will occur after pause() is
  // called and after any "resumed" messages are sent. Note that it is possible
  // that a packet is still in the middle of being transmitted.
  bool isPaused() const {
    return paused_;
  }

  // Resumes continuous asynchronous packet sending.
  void resume();

  // Resumes sending, but pauses again after the specified number of packets are
  // sent. Values < 0 will be ignored and a value of zero will resume.
  //
  // If sending is not already paused, only the next n packets will be sent, not
  // including any already in transmission.
  //
  // There are two ways to determine when the packets are done being sent. The
  // first is by polling isTransmitting(). The second is to use a function that
  // receives transmission-complete notifications. It is called when the same
  // conditions checked by isTransmitting() occur. onDoneTransmitting() sets
  // this function.
  void resumeFor(int n);

  // Resumes sending, but pauses again after the specified number of packets are
  // sent. Values < 0 will be ignored and a value of zero will resume.
  //
  // If sending is not already paused, only the next n packets will be sent, not
  // including any already in transmission.
  //
  // When transmitting is done, the given function will be called. This replaces
  // any function set by onDoneTransmitting().
  void resumeFor(int n, void (*doneTXFunc)(Sender *s));

  // Returns the number of packets remaining to be sent before being paused.
  // This will return zero if there are no packets remaining.
  int resumedRemaining() const {
    return resumeCounter_;
  }

  // Returns if we are currently transmitting a packet or we are not currently
  // paused. To wait until transmission is complete after pausing, the following
  // bit of code is useful:
  //     while (isTransmitting()) { yield(); }
  //
  // Note that this will always return true if we are not paused.
  //
  // An alternative to this function is to use onDoneTransmitting() to be
  // notified when transmission is complete.
  bool isTransmitting() const;

  // Sets the function to call when the sender is paused and transmission
  // of the current packet is done. This can be used instead of polling
  // isTransmitting(). The function takes one argument, a pointer to this
  // Sender instance.
  //
  // The function is called when the same conditions checked by isTransmitting()
  // occur. It is called from an ISR.
  void onDoneTransmitting(void (*f)(Sender *s)) {
    doneTXFunc_ = f;
  }

 private:
   // State that tracks what to transmit and when.
  enum class XmitStates {
    kBreak,  // Need to transmit a break
    kData,   // Need to transmit data
    kIdle,   // The end of data for one packet has been reached
  };

  // The minimum allowed packet time for senders, either BREAK plus data,
  // or BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMinDMXPacketTime = 1204;

  // Disables all the UART IRQs so that variables can be accessed concurrently.
  void disableIRQs() const;

  // Enables all the UART IRQs.
  void enableIRQs() const;

  // Completes a sent packet. This increments the packet count, resets the
  // output buffer index, and sets the state to Idle.
  //
  // This is called from an ISR.
  void completePacket();

  // Keeps track of what we're transmitting.
  volatile XmitStates state_;

  // Use this buffer while paused. This way, we don't have to worry about
  // affecting the currently-transmitting packet.
  uint8_t pausedBuf_[kMaxDMXPacketSize];

  volatile uint8_t outputBuf_[kMaxDMXPacketSize];
  int outputBufIndex_;

  // The size of the packet to be sent.
  volatile int packetSize_;

  // The packet refresh rate, in Hz.
  float refreshRate_;
  IntervalTimer refreshRateTimer_;  // Accompanying timer

  // The BREAK-to-BREAK timing, matching the refresh rate.
  // This is specified in microseconds.
  volatile uint32_t breakToBreakTime_;

  // Keeps track of the time since the last break.
  elapsedMicros timeSinceBreak_;

  // For pausing
  volatile bool paused_;
  volatile int resumeCounter_;
  volatile bool transmitting_;  // Indicates whether we are currently
                                // transmitting a packet

  // This is called when we are done transmitting after a resumeFor call.
  void (*volatile doneTXFunc_)(Sender *s);

  // These error ISRs need to access private functions
  friend void uart0_tx_isr();
  friend void uart1_tx_isr();
  friend void uart2_tx_isr();
#ifdef HAS_KINETISK_UART3
  friend void uart3_tx_isr();
#endif  // HAS_KINETISK_UART3
#ifdef HAS_KINETISK_UART4
  friend void uart4_tx_isr();
#endif  // HAS_KINETISK_UART4
#ifdef HAS_KINETISK_UART5
  friend void uart5_tx_isr();
#endif  // HAS_KINETISK_UART5
#ifdef HAS_KINETISK_LPUART0
  friend void lpuart0_tx_isr();
#endif  // HAS_KINETISK_LPUART0
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // QINDESIGN_TEENSYDMX_H_
