/*
 * Library for doing DMX on a Teensy. Note that transmit and receive can't
 * be done on the same serial port.
 *
 * (c) 2017-2020 Shawn Silverman
 */

/*
  Links:
  https://github.com/jimparis/DmxReceiver
  https://forum.pjrc.com/threads/19662-Arduinoesque-overriding-of-core-functionality?highlight=teensydmx
  https://www.holidaycoro.com/v/vspfiles/downloads/UnderstandingDMX.pdf
  [Teensy 3.1/3.2](https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf)
  [Teensy LC](https://www.pjrc.com/teensy/KL26P121M48SF4RM.pdf)
  [Teensy 3.5](https://www.pjrc.com/teensy/K64P144M120SF5RM.pdf)
  [Teensy 3.6](https://www.pjrc.com/teensy/K66P144M180SF5RMV2.pdf)
  [Teensy 4.0/4.1](https://www.pjrc.com/teensy/IMXRT1060RM_rev2.pdf)
*/

#ifndef TEENSYDMX_TEENSYDMX_H_
#define TEENSYDMX_TEENSYDMX_H_

// C++ includes
#include <cstdint>
#include <memory>

// Other includes
#include <Arduino.h>

// Project includes
#include "LPUARTReceiveHandler.h"
#include "LPUARTSendHandler.h"
#include "ReceiveHandler.h"
#include "Responder.h"
#include "SendHandler.h"
#include "UARTReceiveHandler.h"
#include "UARTSendHandler.h"
#include "util/IntervalTimerEx.h"

namespace qindesign {
namespace teensydmx {

// The maximum size of a DMX packet, including the start code.
constexpr int kMaxDMXPacketSize = 513;

// The minimum size of a DMX packet, including the start code. This value is
// used for senders and is a guideline for how many slots will fit in a packet,
// assuming full-speed transmission. This value ensures that the packet
// transmission time does not exceed 1204us.
constexpr int kMinDMXPacketSize = 25;

// The minimum BREAK time allowed by the specification for transmitters,
// in microseconds.
constexpr int kMinTXBreakTime = 92;

// The minimum MAB time allowed by the specification for transmitters,
// in microseconds.
constexpr int kMinTXMABTime = 12;

// TeensyDMX implements either a receiver or transmitter on one of hardware
// serial ports 1-6.
class TeensyDMX {
 public:
  // TeensyDMX is neither copyable nor movable.
  TeensyDMX(const TeensyDMX &) = delete;
  TeensyDMX &operator=(const TeensyDMX &) = delete;

  // Returns the total number of packets received or transmitted since the
  // instance was started. This is reset when begin() is called.
  uint32_t packetCount() const {
    return packetCount_;
  }

 protected:
  // Creates a new DMX receiver or transmitter using the given hardware UART.
  // https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rc-explicit
  explicit TeensyDMX(HardwareSerial &uart);

  ~TeensyDMX() = default;

  // https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rc-zero
  // https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rc-dtor-virtual
  // Don't have to define the destructor.

  // Increments the packet count.
  void incPacketCount() {
    packetCount_++;
  }

  // Resets the packet count to zero.
  void resetPacketCount() {
    packetCount_ = 0;
  }

  HardwareSerial &uart_;
  const int serialIndex_;

 private:
  // Sets up the system for receiving or transmitting DMX on the specified
  // serial port.
  virtual void begin() = 0;

  // Tells the system to stop receiving or transmitting DMX. Call this to
  // clean up.
  virtual void end() = 0;

  // The number of packets sent or received. Subclasses must manage this via
  // incPacketCount() and resetPacketCount().
  // https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rh-protected
  volatile uint32_t packetCount_;
};

// ---------------------------------------------------------------------------
//  Receiver
// ---------------------------------------------------------------------------

// A DMX receiver. This receives packets asynchronously.
class Receiver final : public TeensyDMX {
 public:
  // The latest known packet statistics.
  //
  // Notes on the variables:
  // * Size: The latest packet size.
  // * Is short: Indicates whether the packet duration is shorter than the
  //   minimum allowed, 1196us.
  // * Timestamp: The timestamp of the last received packet, in milliseconds.
  //   This value is the time at which the packet is recognized as a packet and
  //   not at the end of the last stop bit. For example, sometimes a packet is
  //   recognized after a certain timeout, and that time is used as the
  //   timestamp, not the time at the end of the last slot.
  //   This may not be what you expect. Please refer to the
  //   `Receiver::lastPacketTimestamp()` docs for more information.
  // * BREAK-plus-MAB time: This is the sum of the BREAK and MAB times, in
  //   microseconds. It's not possible to determine where the BREAK ends and the
  //   MAB starts without using another pin to watch the RX line. See
  //   `setRXWatchPin` for setting up a connected pin.
  // * BREAK-to-BREAK time: The time between the BREAKs of the last packet and
  //   one previous to it. This may not be set at the same time as the other
  //   variables; it just represents the last known duration. It may be zero if
  //   there is no previous packet. This is in microseconds.
  // * Packet time: The duration of the last packet, from the BREAK start to the
  //   last slot end, in microseconds.
  // * BREAK time: The packet's BREAK time. This will be zero if the RX line is
  //   not being monitored.
  // * MAB time: The packet's MAB time. This will be zero if the RX line is not
  //   being monitored.
  class PacketStats final {
   public:
    // Initializes everything to zero.
    constexpr PacketStats()
        : size(0),
          isShort(false),
          timestamp(0),
          breakPlusMABTime(0),
          breakToBreakTime(0),
          frameTimestamp(0),
          packetTime(0),
          breakTime(0),
          mabTime(0),
          extraSize(0),
          nextBreakPlusMABTime(0),
          nextBreakTime(0),
          nextMABTime(0) {}

    ~PacketStats() = default;

    // Support common use of this object
    PacketStats(const PacketStats &) = default;
    PacketStats(PacketStats &&) = default;
    PacketStats &operator=(const PacketStats &) = default;
    PacketStats &operator=(PacketStats &&) = default;

    int size;                   // Packet size
    bool isShort;               // Indicates whether the packet is shorter than
                                // kMinDMXPacketTime
    uint32_t timestamp;         // Timestamp at packet completion, in ms
    uint32_t breakPlusMABTime;  // Sum of BREAK and MAB times, in microseconds
    uint32_t breakToBreakTime;  // Time between BREAKs, in microseconds
    uint32_t frameTimestamp;    // BREAK timestamp, in microseconds
    uint32_t packetTime;        // Packet time, from BREAK start to slot end,
                                // in microseconds

    // The following are only set when there's an RX monitoring pin
    uint32_t breakTime;  // BREAK time, in microseconds
    uint32_t mabTime;    // MAB time, in microseconds

   private:
    // An accumulator for extra bytes beyond the max. packet length.
    // This is private for now because the value may not be in sync
    // by the time a user retrieves the packet info.
    // Note that the packet size may be less than the maximum when this
    // starts incrementing because a Responder may have cut the packet
    // off early. Thus, an actual overflow occurs when
    // size + extraSize >= kMaxDMXPacketSize.
    int extraSize;  // Size beyond kMaxDMXPacketSize

    // Store the 'next' values to solve the one-ahead problem of completing the
    // packet on the next BREAK (or timeout or size limit)
    uint32_t nextBreakPlusMABTime;
    uint32_t nextBreakTime;
    uint32_t nextMABTime;

    friend class Receiver;
  };

  // The latest error counts.
  //
  // Notes on the variables:
  // * Packet timeout count: Total number of packet timeouts.
  // * Framing error count: Total number of framing errors encountered. This
  //   includes BREAKs that are too short.
  // * Short packet count: Total number of packets that were too short.
  // * Long packet count: Total number of packets that were too long.
  class ErrorStats final {
   public:
    // Initializes everything to zero.
    constexpr ErrorStats()
        : packetTimeoutCount(0),
          framingErrorCount(0),
          shortPacketCount(0),
          longPacketCount(0) {}

    ~ErrorStats() = default;

    // Support common use of this object
    ErrorStats(const ErrorStats &) = default;
    ErrorStats(ErrorStats &&) = default;
    ErrorStats &operator=(const ErrorStats &) = default;
    ErrorStats &operator=(ErrorStats &&) = default;

    uint32_t packetTimeoutCount;
    uint32_t framingErrorCount;
    uint32_t shortPacketCount;
    uint32_t longPacketCount;
  };

  // Creates a new receiver and uses the given UART for communication.
  explicit Receiver(HardwareSerial &uart);

  // Destructs Receiver. This calls `end()`.
  ~Receiver();

  // Sets whether to enable or disable the TX driver for the serial port. This
  // can be set anytime. If the receiver is currently in operation, enabling
  // this will cause an 11-bit idle character to be queued for output and the
  // line to remain high after that.
  //
  // The default is to have the transmitter enabled. Note that if any responders
  // cause data to be sent then the transmitter must be enabled manually if it
  // has been disabled; it is not enabled automatically.
  void setTXEnabled(bool flag);

  // Starts up the serial port. This resets all the stats.
  //
  // Call setSetTXNotRXFunc() to set an appropriate pin toggle function before
  // calling begin(). If one is set, this will call it to enable receive.
  void begin() override;

  void end() override;

  // Sets the feature that keeps short packets. These are packets that are
  // shorter than the minimum duration, 1196us. See `readPacket` and
  // `packetStats()` for more information.
  //
  // This feature is disabled by default.
  void setKeepShortPackets(bool flag) {
    keepShortPackets_ = flag;
  }

  // Returns whether short packets are kept.
  bool isKeepShortPackets() const {
    return keepShortPackets_;
  }

  // Reads all or part of the latest packet into buf. This returns zero if len
  // is negative or zero, or if startChannel is negative or beyond
  // `kMaxDMXPacketSize`. This only reads up to the end of the packet if
  // `startChannel+len` would go past the end.
  //
  // This will return the number of bytes actually read into `buf`, -1 if there
  // is no packet available since the last call to this function, or zero if
  // `len` is negative or zero, or if the requested data is outside the range of
  // the received packet. Differentiating between -1 and zero allows the caller
  // to determine whether there was no packet received or a packet was received
  // and did not contain the requested data.
  //
  // The values starting at `startChannel` will be stored starting at index zero
  // in `buf`. `buf` must have a size of at least `len` bytes.
  //
  // Note that this returns the latest data received, even if the receiver has
  // been stopped.
  //
  // If the optional parameter, `stats`, is set to non-NULL, then the latest
  // packet statistics are stored in that object, regardless of this function's
  // return value. The values are read atomically with the latest packet data.
  // This is an advantage over 'packetStats()`.
  //
  // Short packets, packets that don't meet a minimum duration, are normally
  // discarded, but they can be kept by enabling the feature with the
  // `setKeepShortPackets` function. If they are kept, then the
  // `PacketStats::isShort` variable will be set to `true` for these packets and
  // `false` otherwise.
  int readPacket(uint8_t *buf, int startChannel, int len,
                 PacketStats *stats = nullptr);

  // Gets the latest value received for one channel. The start code can be read
  // at channel zero.
  //
  // If the channel is out of range for the last packet or there is no data then
  // this will return zero. The optional `rangeError` parameter can be set to
  // something non-NULL to indicate whether a zero return value meant that there
  // was a range error.
  //
  // Note that this returns the latest value received, even if the receiver has
  // been stopped.
  uint8_t get(int channel, bool *rangeError = nullptr) const;

  // Gets the latest 16-bit value received at the given channel. This reads the
  // value in big-endian order.
  //
  // If the channel is out of range for the last packet, there is no data, or
  // the 16-bit value would span a channel that doesn't exist in the last
  // packet, then this will return zero. The optional `rangeError` parameter can
  // be set to something non-NULL to indicate whether a zero return value meant
  // that there was a range error.
  //
  // Note that this returns the latest value received, even if the receiver has
  // been stopped.
  uint16_t get16Bit(int channel, bool *rangeError = nullptr) const;

  // Returns the latest packet statistics. These are reset when the receiver is
  // started or restarted.
  //
  // Please refer to the `PacketStats` docs for more information.
  //
  // Note that the values returned here may not apply to the data read from the
  // most recent or next packet received. Use `readPacket` to retrieve
  // them atomically.
  PacketStats packetStats() const;

  // Returns the timestamp of the last received packet. Under the covers,
  // `millis()` is noted when a packet is recognized as a packet and not at the
  // end of the last stop bit.
  //
  // Note that this may not indicate freshness of the channels you're interested
  // in because they may not have been a part of the last packet received. i.e.
  // the last packet received may have been smaller than required.
  //
  // Use of this function is discouraged in favor of making a note of the time
  // inside the code that checks for the value returned from `readPacket` being
  // at least equal to the number of channels requested. Use is only suggested
  // in very simple use cases where a timestamp is needed for *any* packet
  // containing data, not necessarily desired data.
  //
  // Don't use this function to detect timeouts for data on specific channels.
  // For example, unplugging a cable might result in a valid packet, but not
  // containing the channels you need. Using this value to detect the last valid
  // data received would give a value that's later than the true value.
  //
  // This returns the same value as `packetStats().timestamp`.
  uint32_t lastPacketTimestamp() const;

  // Sets the responder for the supplied start code. This holds on to the
  // pointer, so callers should take care to not free the object before this
  // Receiver is freed or the responder for the start code is set to `nullptr`.
  //
  // This will replace any responder having the same start code and returns
  // the replaced pointer. Note that this will return `nullptr` if no responder
  // was replaced.
  //
  // Setting the responder for a start code to `nullptr` will remove any
  // previously-set responder for that start code.
  //
  // This function dynamically allocates memory. On small systems, the memory
  // may not be available, so it is possible that this will silently fail. To
  // detect this condition, you can check `errno` for the `ENOMEM` condition. If
  // this case occurs, then all responders can be considered wiped out; this
  // includes all previously-set responders. This will return `nullptr` if this
  // happens.
  //
  // Responder functions are called from an ISR.
  Responder *setResponder(uint8_t startCode, Responder *r);

  // Sets the `setTXNotRX` implementation function. This should be called before
  // calling `begin()`.
  //
  // The given function may be called from an ISR.
  void setSetTXNotRXFunc(void (*f)(bool flag)) {
    setTXNotRXFunc_ = f;
  }

  // Sets the pin that monitors the RX line to determine BREAK and MAB timing.
  // Set to a negative value to unset. The default is unset.
  //
  // Don't forget to configure the pin as an input.
  void setRXWatchPin(int pin);

  // Returns the RX monitoring pin. This will return a negative value if unset.
  int rxWatchPin() const {
    return rxWatchPin_;
  }

  // Returns whether this is considered to be connected to a DMX transmitter. A
  // connection is considered to have been broken if a timeout was detected or a
  // BREAK plus Mark after BREAK (MAB) was too short.
  bool connected() const {
    return connected_;
  }

  // Sets the function to call when the connection state changes. This can be
  // used instead of polling `connected()`. The function takes one argument, a
  // pointer to this Receiver instance.
  //
  // The function is called when the same conditions checked by `connected()`
  // occur. It is called from an ISR.
  void onConnectChange(void (*f)(Receiver *r)) {
    connectChangeFunc_ = f;
  }

  // Returns the latest error statistics. These are reset when the receiver is
  // started or restarted.
  //
  // Please refer to the `ErrorStats` docs for more information.
  ErrorStats errorStats() const;

 private:
  // State that tracks where we are in the receive process.
  enum class RecvStates {
    kBreak,     // BREAK
    kMAB,       // Mark after BREAK
    kData,      // Packet data
    kDataIdle,  // Received data is beyond the max. packet size
    kIdle,      // The end of data for one packet has been reached
  };

  // Interrupt lock that uses RAII to disable and enable the UART interrupts.
  class Lock final {
   public:
    explicit Lock(const Receiver &r) : r_(r) {
      r_.disableIRQs();
    }

    ~Lock() {
      r_.enableIRQs();
    }

   private:
    const Receiver &r_;
  };

  // The maximum allowed packet time for receivers, both BREAK plus data and
  // BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMaxDMXPacketTime = 1250000;

  // The minimum allowed packet time for receivers, both BREAK plus data and
  // BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMinDMXPacketTime = 1196;

  // The maximum allowed IDLE and Mark before BREAK (MBB) time,
  // in microseconds, exclusive.
  static constexpr uint32_t kMaxDMXIdleTime = 1000000;

  // Disables all the UART IRQs so that variables can be accessed concurrently.
  // The IRQs are not disabled if `began_` is `false`.
  void disableIRQs() const;

  // Enables all the UART IRQs.
  // The IRQs are not enabled if `began_` is `false`.
  void enableIRQs() const;

  // Called when the connection state changes.
  // This may be called from an ISR.
  void setConnected(bool flag);

  // Does these things
  // 1. If there's data:
  //    1. Makes a new packet available and sets the packet stats, and
  //    2. Calls any associated responder.
  // 2. Sets the state to either `kDataIdle` or `kIdle`.
  // This is called from an ISR.
  void completePacket(RecvStates newState);

  // Called when the idle timer expires.
  void idleTimerCallback();

  // Look for potential packet timeouts when an IDLE condition was detected.
  // This is called from an ISR.
  void receiveIdle(uint32_t eventTime);

  // A potential BREAK has just been received.
  // This is called from an ISR.
  void receivePotentialBreak(uint32_t eventTime);

  // An invalid start-of-BREAK was received. There were non-zero bytes in the
  // framing error.
  // This is called from an ISR.
  void receiveBadBreak();

  // Receives a byte. The `eopTime` parameter is the timestamp of the end of the
  // character, in microseconds.
  // This is called from an ISR.
  void receiveByte(uint8_t b, uint32_t eopTime);

  // ISR functions.
  void rxPinFell_isr();
  void rxPinRose_isr();

  // Sets whether to enable or disable TX or RX through some external means.
  // This is needed when responding to a received message and transmission
  // needs to occur. This is also called at the end of `begin()` with `false`
  // to enable RX.
  //
  // This does nothing if there is no TX/!RX function set.
  void setTXNotRX(bool flag) const {
    void (*f)(bool) = setTXNotRXFunc_;
    if (f != nullptr) {
      f(flag);
    }
  }

  std::unique_ptr<ReceiveHandler> receiveHandler_;

  // Whether the transmitter is or should be enabled.
  volatile bool txEnabled_;

  // Tracks whether the system has been configured.
  volatile bool began_;

  // Keeps track of what we're receiving.
  volatile RecvStates state_;

  // Features
  volatile bool keepShortPackets_;

  // Receive buffers
  uint8_t buf1_[kMaxDMXPacketSize];
  uint8_t buf2_[kMaxDMXPacketSize];
  uint8_t *activeBuf_;
  // Read-only shared memory buffer, make const volatile
  // https://embeddedgurus.com/barr-code/2012/01/combining-cs-volatile-and-const-keywords/
  const uint8_t *volatile inactiveBuf_;
  int activeBufIndex_;

  // The size of the last received packet. This will be set to zero when
  // `readPacket` reads data. The last packet size does not get set to zero when
  // packet data is read.
  volatile int packetSize_;

  // Holds statistics about the last packet. This replaces `lastPacketSize_` and
  // `packetTimestamp_`, and adds other information.
  PacketStats packetStats_;

  // Current and last BREAK start times, in microseconds. The last start time is
  // zero if we can consider that there's been no prior packet, and the current
  // start time isn't set until it's confirmed that there's been a valid BREAK.
  uint32_t lastBreakStartTime_;
  uint32_t breakStartTime_;

  // Last time a slot ended, in microseconds.
  uint32_t lastSlotEndTime_;

  // Indicates whether we are connected to a DMX transmitter. Disconnection is
  // considered to have occurred when a timeout or framing error is detected.
  // Connection is considered to have occurred when a valid BREAK and at least
  // one byte of data with valid timings have been received.
  volatile bool connected_;

  // This is called when the connection state changes.
  void (*volatile connectChangeFunc_)(Receiver *r);

  // Error stats.
  ErrorStats errorStats_;

  // Responders state
  std::unique_ptr<Responder *[]> responders_;
  int responderCount_;
  std::unique_ptr<uint8_t[]> responderOutBuf_;
  int responderOutBufLen_;

  // Function for enabling/disabling RX and TX.
  void (*volatile setTXNotRXFunc_)(bool flag);

  // Pin that monitors the RX line to determing BREAK and MAB timing. -1 if the
  // pin is not set.
  volatile int rxWatchPin_;

  // Things measured by the RX watch pin interrupt. Sometimes these values can
  // be inferred even without monitoring the RX pin.
  volatile bool seenMABStart_;  // Tracks the RX state transitions for measuring
                                // a BREAK
  uint32_t mabStartTime_;       // When we've seen the pin rise

  // Timer for tracking IDLE timeouts and for timing sending a responder BREAK.
  util::IntervalTimerEx intervalTimer_;

#if defined(__IMXRT1062__) || defined(__IMXRT1052__) || defined(__MK66FX1M0__)
  friend class LPUARTReceiveHandler;
#endif
#if defined(KINETISK) || defined(KINETISL)
  friend class UARTReceiveHandler;
#endif

  // RX pin change ISRs
  friend void rxPinFellSerial0_isr();
  friend void rxPinRoseSerial0_isr();
  friend void rxPinFellSerial1_isr();
  friend void rxPinRoseSerial1_isr();
  friend void rxPinFellSerial2_isr();
  friend void rxPinRoseSerial2_isr();
  friend void rxPinFellSerial3_isr();
  friend void rxPinRoseSerial3_isr();
  friend void rxPinFellSerial4_isr();
  friend void rxPinRoseSerial4_isr();
  friend void rxPinFellSerial5_isr();
  friend void rxPinRoseSerial5_isr();
  friend void rxPinFellSerial6_isr();
  friend void rxPinRoseSerial6_isr();
#if defined(__IMXRT1062__)
  friend void rxPinFellSerial7_isr();
  friend void rxPinRoseSerial7_isr();
#endif

  // These error ISRs need to access private functions
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
  friend void uart0_rx_isr();
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
  friend void uart1_rx_isr();
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
  friend void uart2_rx_isr();
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
  friend void uart3_rx_isr();
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
  friend void uart4_rx_isr();
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
  friend void uart5_rx_isr();
#endif  // HAS_KINETISK_UART5

#if defined(HAS_KINETISK_LPUART0)
  friend void lpuart0_rx_isr();
#endif  // HAS_KINETISK_LPUART0

#ifdef IMXRT_LPUART6
  friend void lpuart6_rx_isr();
#endif  // IMXRT_LPUART6

#ifdef IMXRT_LPUART4
  friend void lpuart4_rx_isr();
#endif  // IMXRT_LPUART4

#ifdef IMXRT_LPUART2
  friend void lpuart2_rx_isr();
#endif  // IMXRT_LPUART2

#ifdef IMXRT_LPUART3
  friend void lpuart3_rx_isr();
#endif  // IMXRT_LPUART3

#ifdef IMXRT_LPUART8
  friend void lpuart8_rx_isr();
#endif  // IMXRT_LPUART8

#ifdef IMXRT_LPUART1
  friend void lpuart1_rx_isr();
#endif  // IMXRT_LPUART1

#ifdef IMXRT_LPUART7
  friend void lpuart7_rx_isr();
#endif  // IMXRT_LPUART7

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))
  friend void lpuart5_rx_isr();
#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)
};

// ---------------------------------------------------------------------------
//  Sender
// ---------------------------------------------------------------------------

// A DMX transmitter. This sends packets asynchronously.
class Sender final : public TeensyDMX {
 public:
  // Creates a new transmitter and uses the given UART for communication.
  explicit Sender(HardwareSerial &uart);

  // Destructs Sender. This calls `end()`.
  ~Sender();

  // Starts up the serial port. This resets all the stats.
  void begin() override;

  // Ends sending. Note that this does not wait for the current packet to
  // finish. To complete the current packet, pause and then check if
  // transmission is still active. See `pause()` and `isTransmitting()`.
  void end() override;

  // Sets the BREAK time, in microseconds, for when a timer is selected to
  // perform the timing. Note that if the timing could not be achieved due to an
  // internal problem, for example a timer is unavailable, then the BREAK/MAB
  // serial parameters will be used instead.
  //
  // The BREAK time will be pretty accurate, but, due to the inaccuracy of the
  // default IntervalTimer API, it won't be exact. (It doesn't allow precisely
  // starting an action, in this case starting the BREAK.)
  //
  // Note that the specification states that the BREAK time must be at
  // least 92us. See `kMinTXBreakTime`.
  //
  // The default duration is 180us.
  void setBreakTime(uint32_t t);

  // Returns this sender's BREAK time, in microseconds. The value returned is
  // dependent on whether a timer or serial parameters are being used to
  // generate the timing.
  //
  // If a timer is being used then the actual time may be slightly different,
  // due to the inaccuracy of the default IntervalTimer API. (It doesn't allow
  // precisely starting an action, in this case starting the BREAK.)
  //
  // If serial parameters are being used then the actual time will likely match
  // closely with the return value.
  uint32_t breakTime() const;

  // Sets the MAB time, in microseconds, for when a timer is selected to perform
  // the timing. Note that if the BREAK/MAB timing could not be achieved due to
  // an internal problem, for example if a timer is unavailable, then the
  // BREAK/MAB serial parameters will be used instead.
  //
  // Due to some system timing, the actual MAB time may be longer.
  //
  // Note that the specification states that the MAB time must be at least 12us
  // and less than 1s. See `kMinTXMABTime`.
  //
  // The default duration is 20us.
  void setMABTime(uint32_t t);

  // Returns this sender's Mark after BREAK (MAB) time, in microseconds. The
  // value returned is dependent on whether a timer or serial parameters are
  // being used to generate the timing. In both cases, the actual time will
  // likely be larger than the return value due to some UART intricacies.
  uint32_t mabTime() const;

  // Sets the BREAK/MAB serial port parameters. The parameters will only be
  // applied when the transmitter is not running. To apply the parameters, the
  // transmitter must be started or restarted.
  //
  // This will return whether the parameters are valid. If this returns false
  // then the parameters will not be set; they will be set otherwise. Invalid
  // parameters include:
  // * A baud rate of zero
  // * Formats that specify TXINV
  // * Unknown formats
  //
  // The default parameters are 50000 baud and 8N1, approximately a 180us BREAK
  // and 20us MAB.
  bool setBreakSerialParams(uint32_t baud, uint32_t format);

  // Returns the currently-set BREAK baud rate.
  uint32_t breakSerialBaud() const {
    return breakBaud_;
  }

  // Returns the currently-set BREAK serial format.
  uint32_t breakSerialFormat() const {
    return breakFormat_;
  }

  // Sets whether to use a timer or serial parameters to achieve the BREAK and
  // MAB timings.
  //
  // The default is to use serial parameters and not a timer.
  void setBreakUseTimerNotSerial(bool flag) {
    breakUseTimer_ = flag;
  }

  // Returns whether a timer or serial parameters are being used to achieve the
  // BREAK and MAB timings.
  bool isBreakUseTimerNotSerial() const {
    return breakUseTimer_;
  }

  // Sets the transmit packet size, in number of channels plus the start code.
  // This returns `false` if the size is not in the range 1-513. Otherwise, this
  // returns `true`.
  //
  // When the maximum refresh rate is used, the packet size should be >= 25 so
  // that the total packet time does not fall below 1204us, per the ANSI E1.11
  // DMX specification. However, smaller packets can be sent if the refresh rate
  // is decreased.
  //
  // These limits are contained in `kMaxDMXPacketSize` and `kMinDMXPacketSize`.
  //
  // For example, if the packet size is set to 25, then the channels can range
  // from 0 to 24, inclusive, with channel zero containing the start code and
  // slots 1-24 containing the remainder of the packet data.
  //
  // The default size is 513.
  bool setPacketSize(int size);

  // Returns the current packet size.
  int packetSize() const {
    return activePacketSize_;
  }

  // Sets a channel's value. Channel zero represents the start code. The start
  // code should really be zero, but it can be changed here. This also affects
  // the packet currently being transmitted.
  //
  // Values set here are 'sticky'. In other words, the same values are
  // transmitted until changed. To set a value, this only needs to be
  // called once.
  //
  // If the channel is not in the range 0-512 then this returns `false`.
  // Otherwise, this returns `true`. Note that it is possible to set channels
  // outside the range of the packet size, but these values will not be sent.
  //
  // For example, if the packet size is 25 and the channel is anywhere in the
  // range 25-512, then the value will be set internally but will not be
  // transmitted until the packet size changes via `setPacketSize`.
  //
  // After pausing with `pause()`, it is necessary to wait until transmission is
  // finished before setting channel values.
  bool set(int channel, uint8_t value);

  // Sets a 16-bit value at the specified channel. This stores the value in
  // big-endian order. Channel zero represents the start code.
  //
  // Values set here are 'sticky'. In other words, the same values are
  // transmitted until changed. To set a value, this only needs to be
  // called once.
  //
  // If the channel is not in the range 0-511 then this returns `false`.
  // Otherwise, this returns `true`. Note that it is possible to set channels
  // outside the range of the packet size, but these values will not be sent.
  //
  // For example, if the packet size is 25 and the channel is anywhere in the
  // range 24-511, then the value will be set internally but will not be
  // completely transmitted until the packet size changes via `setPacketSize`.
  // In this example, if the channel is 24 then the value would be split between
  // a channel that's sent and one that isn't.
  //
  // After pausing with `pause()`, it is necessary to wait until transmission is
  // finished before setting channel values.
  bool set16Bit(int channel, uint16_t value);

  // Sets the values for a range of channels. This also affects the packet
  // currently being transmitted. The behaviour is atomic.
  //
  // Values set here are 'sticky'. In other words, the same values are
  // transmitted until changed. To set some values, this only needs to be
  // called once.
  //
  // This returns `false` if any part of the channel range is not in the range
  // 0-512, if the length is negative, or if `values` is NULL. Otherwise, this
  // returns `true`. The upper limit is equal to `kDMXMaxPacketSize-1`.
  //
  // See the other 'set' function for more information about setting
  // values outside the range of the current packet size (if the size
  // is less than 513).
  //
  // After pausing with `pause()`, it is necessary to wait until transmission is
  // finished before setting channel values.
  bool set(int startChannel, const uint8_t *values, int len);

  // Sets 16-bit values for a range of channels. See `set` for more information.
  bool set16Bit(int startChannel, const uint16_t *values, int len);

  // Clears all channels to zero. The behaviour is atomic.
  void clear();

  // Fills all channels in the specified range to the given value.
  // The behaviour is atomic.
  //
  // This returns `false` if any part of the channel range is not in the range
  // 0-512, or if the length is negative. Otherwise, this returns `true`. The
  // upper limit is equal to `kDMXMaxPacketSize-1`.
  bool fill(int startChannel, int len, uint8_t value);

  // Sets the packet refresh rate. This returns `false` for negative and NaN
  // values, and `true` otherwise. The default is INFINITY, indicating "as fast
  // as possible".
  //
  // If the rate is too high then this will simply transmit as fast as possible.
  // Transmitting as fast as possible is also the default.
  //
  // If the rate is zero then no packets will be sent. However, the serial
  // transmitter will still be enabled. A rate of zero is not equivalent to
  // calling `end()`.
  //
  // If the new rate is non-zero and the former rate is zero then this will call
  // `end()` and then `begin()`.
  //
  // For rates slower than the maximum, this uses one of the periodic
  // timers internally.
  //
  // Note that the specification states that the BREAK-to-BREAK time must be at
  // most 1s. This means that the minimum refresh rate is 1Hz. The minimum
  // theoretical 513-slot time is 22.676us, so the maximum theoretical rate is
  // ~44.099Hz. The minimum allowed BREAK-to-BREAK time is 1204us, so for a
  // packet having all minimum timings, this means that a minimum 25-slot packet
  // can be sent at a refresh rate of ~830.56Hz.
  bool setRefreshRate(float rate);

  // Returns the packet refresh rate. The default is INFINITY, indicating
  // "as fast as possible".
  float refreshRate() const {
    return refreshRate_;
  }

  // Pauses the ansynchronous packet sending. This allows information to be
  // inserted at a specific point. The pause actually occurs after finishing
  // transmission of the current packet.
  //
  // Note that the current packet needs to complete before setting channel data
  // with one of the 'set' functions. Use them freely after waiting for
  // transmission to finish. See `isTransmitting()` and `onDoneTransmitting`.
  //
  // Also note that this does not change the number of resumed packets
  // remaining.
  //
  // This is useful, for example, for System Information Packets (SIP), where
  // checksum data needs to be applied to the preceding packet.
  void pause() {
    paused_ = true;
  }

  // Returns whether we are currently paused. This will occur after `pause()` is
  // called and after any "resumed" messages are sent. Note that it is possible
  // that a packet is still in the middle of being transmitted even if this
  // returns `true`; it only reflects the current setting.
  bool isPaused() const {
    return paused_;
  }

  // Resumes continuous asynchronous packet sending.
  void resume();

  // Resumes sending, but pauses again after the specified number of packets are
  // sent. A value of zero will resume. This will return `false` for values < 0
  // and `true` for all other values. In other words, this will return `true`
  // when sending is resumed.
  //
  // If sending is not already paused, only the next n packets will be sent, not
  // including any already in transmission.
  //
  // There are two ways to determine when the packets are done being sent. The
  // first is by polling `isTransmitting()`. The second is to use a function
  // that receives transmission-complete notifications. It is called when the
  // same conditions checked by `isTransmitting()` occur.
  //
  // This uses any function set by `onDoneTransmitting`.
  bool resumeFor(int n);

  // Resumes sending, but pauses again after the specified number of packets are
  // sent. A value of zero will resume. This will return `false` for values < 0
  // and `true` for all other values. In other words, this will return `true`
  // when sending is resumed.
  //
  // If sending is not already paused, only the next n packets will be sent, not
  // including any already in transmission.
  //
  // When transmitting is done, the given function will be called. This replaces
  // any function set by `onDoneTransmitting`. Setting the function to `nullptr`
  // is valid.
  bool resumeFor(int n, void (*doneTXFunc)(Sender *s));

  // Returns the number of packets remaining to be sent before being paused.
  // This will return zero if there are no packets remaining.
  int resumedRemaining() const {
    return resumeCounter_;
  }

  // Returns if we are currently transmitting a packet or we are not currently
  // paused. To wait until transmission is complete after pausing, the following
  // bit of code is useful:
  // ```
  //     while (isTransmitting()) { yield(); }
  // ```
  //
  // Note that this will always return `true` if we are not paused.
  //
  // An alternative to this function is to use `onDoneTransmitting` to be
  // notified when transmission is complete.
  bool isTransmitting() const;

  // Sets the function to call when the sender is paused and transmission
  // of the current packet is done. This can be used instead of polling
  // `isTransmitting()`. The function takes one argument, a pointer to this
  // Sender instance.
  //
  // The function is called when the same conditions checked by
  // `isTransmitting()` occur. It is called from an ISR.
  //
  // The function may be set to `nullptr`.
  void onDoneTransmitting(void (*f)(Sender *s)) {
    doneTXFunc_ = f;
  }

 private:
  // State that tracks what to transmit and when.
  enum class XmitStates {
    kBreak,  // Need to transmit a BREAK
    kMAB,    // Need to transmit a MAB
    kData,   // Need to transmit data
    kIdle,   // The end of data for one packet has been reached
  };

  // Interrupt lock that uses RAII to disable and enable the UART interrupts.
  class Lock final {
   public:
    explicit Lock(const Sender &s) : s_(s) {
      s_.disableIRQs();
    }

    ~Lock() {
      s_.enableIRQs();
    }

   private:
    const Sender &s_;
  };

  std::unique_ptr<SendHandler> sendHandler_;

  // The minimum allowed packet time for senders, either BREAK plus data,
  // or BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMinDMXPacketTime = 1204;

  // Disables all the UART IRQs so that variables can be accessed concurrently.
  // The IRQs are not disabled if `began_` is `false`.
  void disableIRQs() const;

  // Enables all the UART IRQs.
  // The IRQs are not enabled if `began_` is `false`.
  void enableIRQs() const;

  // Completes a sent packet. This increments the packet count, resets the
  // output buffer index, and sets the state to `kIdle`.
  //
  // This is called from an ISR.
  void completePacket();

  // Tracks whether the system has been configured.
  volatile bool began_;

  // Keeps track of what we're transmitting.
  volatile XmitStates state_;

  // Output buffers
  volatile uint8_t activeBuf_[kMaxDMXPacketSize];
  volatile uint8_t inactiveBuf_[kMaxDMXPacketSize];
  volatile int inactiveBufIndex_;

  // BREAK and MAB times
  uint32_t breakTime_;
  uint32_t mabTime_;

  // Adjusted for the real world
  volatile uint32_t adjustedBreakTime_;
  volatile uint32_t adjustedMABTime_;

  // BREAK serial parameters
  uint32_t breakBaud_;
  uint32_t breakFormat_;
  volatile bool breakUseTimer_;  // Whether to use a timer or serial parameters
                                 // for BREAK/MAB times

  // The size of the packet to be sent.
  volatile int activePacketSize_;
  volatile int inactivePacketSize_;

  // The packet refresh rate, in Hz.
  float refreshRate_;
  util::IntervalTimerEx intervalTimer_;  // General purpose timer

  // The BREAK-to-BREAK timing, matching the refresh rate.
  // This is specified in microseconds.
  volatile uint32_t breakToBreakTime_;

  // Keeps track of the last BREAK start time, in milliseconds. This is for
  // refresh rate timing.
  uint32_t breakStartTime_;

  // For pausing
  volatile bool paused_;
  volatile int resumeCounter_;
  volatile bool transmitting_;  // Indicates whether we are currently
                                // transmitting a packet

  // This is called when we are done transmitting after a `resumeFor` call.
  void (*volatile doneTXFunc_)(Sender *s);

#if defined(__IMXRT1062__) || defined(__IMXRT1052__) || defined(__MK66FX1M0__)
  friend class LPUARTSendHandler;
#endif
#if defined(KINETISK) || defined(KINETISL)
  friend class UARTSendHandler;
#endif

  // These error ISRs need to access private functions
#if defined(HAS_KINETISK_UART0) || defined(HAS_KINETISL_UART0)
  friend void uart0_tx_isr();
#endif  // HAS_KINETISK_UART0 || HAS_KINETISL_UART0

#if defined(HAS_KINETISK_UART1) || defined(HAS_KINETISL_UART1)
  friend void uart1_tx_isr();
#endif  // HAS_KINETISK_UART1 || HAS_KINETISL_UART1

#if defined(HAS_KINETISK_UART2) || defined(HAS_KINETISL_UART2)
  friend void uart2_tx_isr();
#endif  // HAS_KINETISK_UART2 || HAS_KINETISL_UART2

#if defined(HAS_KINETISK_UART3)
  friend void uart3_tx_isr();
#endif  // HAS_KINETISK_UART3

#if defined(HAS_KINETISK_UART4)
  friend void uart4_tx_isr();
#endif  // HAS_KINETISK_UART4

#if defined(HAS_KINETISK_UART5)
  friend void uart5_tx_isr();
#endif  // HAS_KINETISK_UART5

#if defined(HAS_KINETISK_LPUART0)
  friend void lpuart0_tx_isr();
#endif  // HAS_KINETISK_LPUART0

#ifdef IMXRT_LPUART6
  friend void lpuart6_tx_isr();
#endif  // IMXRT_LPUART6

#ifdef IMXRT_LPUART4
  friend void lpuart4_tx_isr();
#endif  // IMXRT_LPUART4

#ifdef IMXRT_LPUART2
  friend void lpuart2_tx_isr();
#endif  // IMXRT_LPUART2

#ifdef IMXRT_LPUART3
  friend void lpuart3_tx_isr();
#endif  // IMXRT_LPUART3

#ifdef IMXRT_LPUART8
  friend void lpuart8_tx_isr();
#endif  // IMXRT_LPUART8

#ifdef IMXRT_LPUART1
  friend void lpuart1_tx_isr();
#endif  // IMXRT_LPUART1

#ifdef IMXRT_LPUART7
  friend void lpuart7_tx_isr();
#endif  // IMXRT_LPUART7

#if defined(IMXRT_LPUART5) && \
    (defined(__IMXRT1052__) || defined(ARDUINO_TEENSY41))
  friend void lpuart5_tx_isr();
#endif  // IMXRT_LPUART5 && (__IMXRT1052__ || ARDUINO_TEENSY41)
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_TEENSYDMX_H_
