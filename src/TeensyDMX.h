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
  [Teensy 3.1/3.2](https://www.pjrc.com/teensy/K20P64M72SF1RM.pdf)
  [Teensy LC](https://www.pjrc.com/teensy/KL26P121M48SF4RM.pdf)
  [Teensy 3.5](https://www.pjrc.com/teensy/K64P144M120SF5RM.pdf)
  [Teensy 3.6](https://www.pjrc.com/teensy/K66P144M180SF5RMV2.pdf)
  [Teensy 4.0](https://www.pjrc.com/teensy/IMXRT1060RM_rev1.pdf)
*/

#ifndef TEENSYDMX_TEENSYDMX_H_
#define TEENSYDMX_TEENSYDMX_H_

// C++ includes
#include <cstdint>
#include <memory>

// Other includes
#include <Arduino.h>

// Project includes
#include "Responder.h"
#include "util/IntervalTimer.h"

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

  virtual ~TeensyDMX() = default;

  // Sets up the system for receiving or transmitting DMX on the specified
  // serial port.
  virtual void begin() = 0;

  // Tells the system to stop receiving or transmitting DMX. Call this to
  // clean up.
  virtual void end() = 0;

  // Returns the total number of packets received or transmitted since the
  // instance was started. This is reset when begin() is called.
  uint32_t packetCount() const {
    return packetCount_;
  }

 protected:
  // Creates a new DMX receiver or transmitter using the given hardware UART.
  // https://github.com/isocpp/CppCoreGuidelines/blob/master/CppCoreGuidelines.md#Rc-explicit
  explicit TeensyDMX(HardwareSerial &uart);

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
  // * Timestamp: The timestamp of the last received packet, in milliseconds.
  //   This may not be what you expect. Please refer to the
  //   `Receiver::lastPacketTimestamp()` docs for more information.
  // * BREAK-plus-MAB time: This is the sum of the BREAK and MAB times, in
  //   microseconds. It's not possible to determine where the BREAK ends and the
  //   MAB starts without using another pin to watch the RX line. See
  //   `setRXWatchPin` for setting up a connected pin.
  // * BREAK-to-BREAK time: This may not be set at the same time as the other
  //   variables; it just represents the last known duration. This is
  //   in microseconds.
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
          timestamp(0),
          breakPlusMABTime(0),
          breakToBreakTime(0),
          packetTime(0),
          breakTime(0),
          mabTime(0),
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
    uint32_t timestamp;         // Timestamp, in milliseconds
    uint32_t breakPlusMABTime;  // Sum of BREAK and MAB times, in microseconds
    uint32_t breakToBreakTime;  // Time between BREAKs, in microseconds
    uint32_t packetTime;        // Packet time, from BREAK start to slot end,
                                // in microseconds

    // The following are only set when there's an RX monitoring pin
    uint32_t breakTime;  // BREAK time, in microseconds
    uint32_t mabTime;    // MAB time, in microseconds

   private:
    // Volatile copy constructor.
    PacketStats(const volatile PacketStats &other)
        : size(other.size),
          timestamp(other.timestamp),
          breakPlusMABTime(other.breakPlusMABTime),
          breakToBreakTime(other.breakToBreakTime),
          packetTime(other.packetTime),
          breakTime(other.breakTime),
          mabTime(other.mabTime),
          nextBreakPlusMABTime(other.nextBreakPlusMABTime),
          nextBreakTime(other.nextBreakTime),
          nextMABTime(other.nextMABTime) {}

    // Volatile assignment operator. This returns void to avoid a warning
    // of non-use.
    // See: https://stackoverflow.com/questions/13869318/gcc-warning-about-implicit-dereference
    void operator=(const PacketStats &other) volatile {
      size = other.size;
      timestamp = other.timestamp;
      breakPlusMABTime = other.breakPlusMABTime;
      breakToBreakTime = other.breakToBreakTime;
      packetTime = other.packetTime;
      breakTime = other.breakTime;
      mabTime = other.mabTime;
      nextBreakPlusMABTime = other.nextBreakPlusMABTime;
      nextBreakTime = other.nextBreakTime;
      nextMABTime = other.nextMABTime;
    }

    // Other-way volatile assignment operator.
    PacketStats &operator=(const volatile PacketStats &other) {
      size = other.size;
      timestamp = other.timestamp;
      breakPlusMABTime = other.breakPlusMABTime;
      breakToBreakTime = other.breakToBreakTime;
      packetTime = other.packetTime;
      breakTime = other.breakTime;
      mabTime = other.mabTime;
      nextBreakPlusMABTime = other.nextBreakPlusMABTime;
      nextBreakTime = other.nextBreakTime;
      nextMABTime = other.nextMABTime;
      return *this;
    }

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
  class ErrorStats final {
   public:
    // Initializes everything to zero.
    constexpr ErrorStats()
        : packetTimeoutCount(0),
          framingErrorCount(0),
          shortPacketCount(0) {}

    ~ErrorStats() = default;

    // Support common use of this object
    ErrorStats(const ErrorStats &) = default;
    ErrorStats(ErrorStats &&) = default;
    ErrorStats &operator=(const ErrorStats &) = default;
    ErrorStats &operator=(ErrorStats &&) = default;

    uint32_t packetTimeoutCount;
    uint32_t framingErrorCount;
    uint32_t shortPacketCount;

   private:
    // Volatile copy constructor.
    ErrorStats(const volatile ErrorStats &other)
        : packetTimeoutCount(other.packetTimeoutCount),
          framingErrorCount(other.framingErrorCount),
          shortPacketCount(other.shortPacketCount) {}

    // Volatile assignment operator. This returns void to avoid a warning
    // of non-use.
    // See: https://stackoverflow.com/questions/13869318/gcc-warning-about-implicit-dereference
    void operator=(const ErrorStats &other) volatile {
      packetTimeoutCount = other.packetTimeoutCount;
      framingErrorCount = other.framingErrorCount;
      shortPacketCount = other.shortPacketCount;
    }

    friend class Receiver;
  };

  // Creates a new receiver and uses the given UART for communication.
  explicit Receiver(HardwareSerial &uart);

  // Destructs Receiver. This calls end().
  ~Receiver() override;

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

  // Reads all or part of the latest packet into buf. This returns zero if len
  // is negative or zero, or if startChannel is negative or beyond
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
  //
  // Note that this returns the latest data received, even if the receiver has
  // been stopped.
  //
  // If the optional parameter, `stats`, is set to non-NULL, then the latest
  // packet statistics are stored in that object, regardless of this function's
  // return value. The values are read atomically with the latest packet data.
  // This is an advantage over 'packetStats()`.
  int readPacket(uint8_t *buf, int startChannel, int len,
                 PacketStats *stats = nullptr);

  // Gets the latest value received for one channel. The start code can be read
  // at channel zero.
  //
  // If the channel is out of range for the last packet or there is no data then
  // this will return zero.
  //
  // Note that this returns the latest value received, even if the receiver has
  // been stopped.
  uint8_t get(int channel) const;

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
  PacketStats packetStats() const {
    Lock lock{*this};
    return packetStats_;
  }

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
  //
  // This returns the same value as `packetStats().timestamp`.
  uint32_t lastPacketTimestamp() const {
    return packetStats_.timestamp;
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
  //
  // This function dynamically allocates memory. On small systems, the memory
  // may not be available, so it is possible that this will silently fail. To
  // detect this condition, you can check `errno` for the ENOMEM condition. If
  // this case occurs, then all responders can be considered wiped out; this
  // includes all previously-set responders. This will return nullptr if this
  // happens.
  //
  // Responder functions are called from an ISR.
  std::shared_ptr<Responder> setResponder(uint8_t startCode,
                                          std::shared_ptr<Responder> r);

  // Sets the setTXNotRX implementation function. This should be called before
  // calling begin().
  //
  // The given function may be called from an ISR.
  void setSetTXNotRXFunc(void (*f)(bool flag)) {
    setTXNotRXFunc_ = f;
  }

  // Sets the pin that monitors the RX line to determine BREAK and MAB timing.
  // Set to a negative value to unset. The default is unset.
  void setRXWatchPin(int pin);

  // Returns whether this is considered to be connected to a DMX transmitter. A
  // connection is considered to have been broken if a timeout was detected or a
  // BREAK plus Mark after BREAK (MAB) was too short.
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

  // Returns the latest error statistics. These are reset when the receiver is
  // started or restarted.
  //
  // Please refer to the `ErrorStats` docs for more information.
  ErrorStats errorStats() const {
    Lock lock{*this};
    return errorStats_;
  }

 private:
  // State that tracks where we are in the receive process.
  enum class RecvStates {
    kBreak,  // BREAK
    kMAB,    // Mark after BREAK
    kData,   // Packet data
    kIdle,   // The end of data for one packet has been reached
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

  // The maximum allowed packet time for receivers, either BREAK plus data,
  // or BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMaxDMXPacketTime = 1250000;

  // The minimum allowed packet time for receivers, BREAK to BREAK,
  // in microseconds.
  static constexpr uint32_t kMinDMXPacketTime = 1196;

  // The maximum allowed IDLE and Mark Before BREAK (MBB) time,
  // in microseconds, exclusive.
  static constexpr uint32_t kMaxDMXIdleTime = 1000000;

  // Disables all the UART IRQs so that variables can be accessed concurrently.
  // The IRQs are not disabled if began_ is false.
  void disableIRQs() const;

  // Enables all the UART IRQs.
  // The IRQs are not enabled if began_ is false.
  void enableIRQs() const;

  // Called when the connection state changes.
  void setConnected(bool flag);

  // Makes a new packet available and resets state.
  // This is called from an ISR.
  void completePacket();

  // Look for potential packet timeouts when an IDLE condition was detected.
  // This is called from an ISR.
  void receiveIdle();

  // A potential BREAK has just been received.
  // This is called from an ISR.
  void receivePotentialBreak();

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
  // needs to occur. This is also called at the end of begin() with 'false'
  // to enable RX.
  //
  // This does nothing if there is no TX/!RX function set.
  void setTXNotRX(bool flag) const {
    void (*f)(bool) = setTXNotRXFunc_;
    if (f != nullptr) {
      f(flag);
    }
  }

  // Whether the transmitter is or should be enabled.
  volatile bool txEnabled_;

  // Tracks whether the system has been configured.
  volatile bool began_;

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

  // The size of the last received packet. This will be set to zero when
  // readPacket() reads data. lastPacketSize_ does not get set to zero when
  // packet data is read.
  volatile int packetSize_;

  // Holds statistics about the last packet. This replaces lastPacketSize_ and
  // packetTimestamp_, and adds other information.
  volatile PacketStats packetStats_;

  // Current and last BREAK start times, in microseconds. The last start time is
  // zero if we can consider that there's been no prior packet, and the current
  // start time isn't set until it's confirmed that there's been a valid BREAK.
  uint32_t lastBreakStartTime_;
  uint32_t breakStartTime_;

  // Last time a slot ended, in microseconds.
  uint32_t lastSlotEndTime_;

  // Indicates whether we are connected to a DMX transmitter. Disconnection is
  // considered to have occurred when a timeout or framing error is detected.
  volatile bool connected_;

  // This is called when the connection state changes.
  void (*volatile connectChangeFunc_)(Receiver *r);

  // Error stats.
  volatile ErrorStats errorStats_;

  // Responders state
  std::unique_ptr<std::shared_ptr<Responder>[]> responders_;
  int responderCount_;
  std::unique_ptr<uint8_t[]> responderOutBuf_;
  int responderOutBufLen_;

  // Function for enabling/disabling RX and TX.
  void (*volatile setTXNotRXFunc_)(bool flag);

  // Pin that monitors the RX line to determing BREAK and MAB timing. -1 if the
  // pin is not set.
  volatile int rxWatchPin_;

  // Things measured by the RX watch pin interrupt
  volatile int rxChangeState_;  // Tracks the RX state transitions for measuring
                                // a BREAK
  uint32_t rxRiseTime_;

  // Transmit function for the current UART.
  void (*txFunc_)(const uint8_t *b, int len);

  // Transmit BREAK function for the current UART. This accepts both a BREAK
  // time and a Mark after BREAK time.
  void (*txBreakFunc_)(uint32_t breakTime, uint32_t mabTime);

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

#if defined(IMXRT_LPUART5) && defined(__IMXRT1052__)
  friend void lpuart5_rx_isr();
#endif  // IMXRT_LPUART5 && __IMXRT1052__
};

// ---------------------------------------------------------------------------
//  Sender
// ---------------------------------------------------------------------------

// A DMX transmitter. This sends packets asynchronously.
class Sender final : public TeensyDMX {
 public:
  // Creates a new transmitter and uses the given UART for communication.
  explicit Sender(HardwareSerial &uart);

  // Destructs Sender. This calls end().
  ~Sender() override;

  // Starts up the serial port. This resets all the stats.
  void begin() override;

  // Ends sending. Note that this does not wait for the current packet to
  // finish. To complete the current packet, pause and then check if
  // transmission is still active. See pause() and isTransmitting().
  void end() override;

  // Sets the BREAK time, in microseconds. Note that if the timing could not be
  // achieved due to an internal problem then it will be sent as a default
  // of 180us.
  //
  // Note that the specification states that the BREAK time must be at
  // least 92us. See `kMinTXBreakTime`.
  void setBreakTime(uint32_t t);

  // Returns this sender's BREAK time, in microseconds.
  uint32_t breakTime() const {
    return breakTime_;
  }

  // Sets the MAB time, in microseconds. Note that if the timing could not be
  // achieved due to an internal problem then it will be sent as a default of
  // about 20us. Also, due to some system timing, the actual MAB time may
  // be longer.
  //
  // Note that the specification states that the MAB time must be at least 12us
  // and less than 1s. See `kMinTXMABTime`.
  void setMABTime(uint32_t t);

  // Returns this sender's Mark after BREAK time, in microseconds.
  //
  // Note that due to some UART intricacies, the actual time may be longer.
  uint32_t mabTime() const {
    return mabTime_;
  }

  // Sets the transmit packet size, in number of channels plus the start code.
  // This returns false if the size is greater than 513 or negative. Otherwise,
  // this returns true.
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
  bool setPacketSize(int size) {
    if (size < 0 || kMaxDMXPacketSize < size) {
      return false;
    }
    packetSize_ = size;
    return true;
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
  // If the channel is not in the range 0-512 then this returns false.
  // Otherwise, this returns true. Note that it is possible to set channels
  // outside the range of the packet size, but these values will not be sent.
  //
  // For example, if the packet size is 25 and the channel is anywhere in the
  // range 25-512, then the value will be set internally but will not be
  // transmitted until the packet size changes via setPacketSize().
  //
  // After pausing with pause(), it is necessary to wait until transmission is
  // finished before setting channel values.
  bool set(int channel, uint8_t value);

  // Sets a 16-bit value at the specified channel. This stores the value in
  // big-endian order. Channel zero represents the start code.
  //
  // Values set here are 'sticky'. In other words, the same values are
  // transmitted until changed. To set a value, this only needs to be
  // called once.
  //
  // If the channel is not in the range 0-511 then this returns false.
  // Otherwise, this returns true. Note that it is possible to set channels
  // outside the range of the packet size, but these values will not be sent.
  //
  // For example, if the packet size is 25 and the channel is anywhere in the
  // range 24-511, then the value will be set internally but will not be
  // completely transmitted until the packet size changes via setPacketSize().
  // In this example, if the channel is 24 then the value would be split between
  // a channel that's sent and one that isn't.
  //
  // After pausing with pause(), it is necessary to wait until transmission is
  // finished before setting channel values.
  bool set16Bit(int channel, uint16_t value);

  // Sets the values for a range of channels. This also affects the packet
  // currently being transmitted. The behaviour is atomic.
  //
  // Values set here are 'sticky'. In other words, the same values are
  // transmitted until changed. To set some values, this only needs to be
  // called once.
  //
  // This returns false if any part of the channel range is not in the range
  // 0-512, or if the length is negative. Otherwise, this returns true. The
  // upper limit is equal to kDMXMaxPacketSize-1.
  //
  // See the other 'set' function for more information about setting
  // values outside the range of the current packet size (if the size
  // is less than 513).
  //
  // After pausing with pause(), it is necessary to wait until transmission is
  // finished before setting channel values.
  bool set(int startChannel, const uint8_t *values, int len);

  // Sets 16-bit values for a range of channels. See `set` for more information.
  bool set16Bit(int startChannel, const uint16_t *values, int len);

  // Clears all channels to zero. The behaviour is atomic.
  void clear();

  // Sets the packet refresh rate. This returns false for negative and NaN
  // values, and true otherwise. The default is INFINITY, indicating "as fast
  // as possible".
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
  // transmission to finish. See isTransmitting() and onDoneTransmitting().
  //
  // Also note that this does not change the number of resumed packets
  // remaining.
  //
  // This is useful, for example, for System Information Packets (SIP), where
  // checksum data needs to be applied to the preceding packet.
  void pause() {
    paused_ = true;
  }

  // Returns whether we are currently paused. This will occur after pause() is
  // called and after any "resumed" messages are sent. Note that it is possible
  // that a packet is still in the middle of being transmitted even if this
  // returns true; it only reflects the current setting.
  bool isPaused() const {
    return paused_;
  }

  // Resumes continuous asynchronous packet sending.
  void resume();

  // Resumes sending, but pauses again after the specified number of packets are
  // sent. A value of zero will resume. This will return false for values < 0
  // and true for all other values. In other words, this will return true when
  // sending is resumed.
  //
  // If sending is not already paused, only the next n packets will be sent, not
  // including any already in transmission.
  //
  // There are two ways to determine when the packets are done being sent. The
  // first is by polling isTransmitting(). The second is to use a function that
  // receives transmission-complete notifications. It is called when the same
  // conditions checked by isTransmitting() occur.
  //
  // This uses any function set by onDoneTransmitting().
  bool resumeFor(int n);

  // Resumes sending, but pauses again after the specified number of packets are
  // sent. A value of zero will resume. This will return false for values < 0
  // and true for all other values. In other words, this will return true when
  // sending is resumed.
  //
  // If sending is not already paused, only the next n packets will be sent, not
  // including any already in transmission.
  //
  // When transmitting is done, the given function will be called. This replaces
  // any function set by onDoneTransmitting(). Setting the function to nullptr
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
  //
  // The function may be set to nullptr.
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

  // Stored LPUART parameters for quickly setting the baud rate between BREAK
  // and slots. Used for Teensy 3.6 and Teensy 4.
  struct LPUARTParams final {
    uint32_t baud = 0x0f000004;  // 5-bit OSR is 0x0f and 13-bit SBR is 0x0004
    uint32_t stat = 0;
    uint32_t ctrl = 0;
  };

  // The minimum allowed packet time for senders, either BREAK plus data,
  // or BREAK to BREAK, in microseconds.
  static constexpr uint32_t kMinDMXPacketTime = 1204;

  // Disables all the UART IRQs so that variables can be accessed concurrently.
  // The IRQs are not disabled if began_ is false.
  void disableIRQs() const;

  // Enables all the UART IRQs.
  // The IRQs are not enabled if began_ is false.
  void enableIRQs() const;

  // Completes a sent packet. This increments the packet count, resets the
  // output buffer index, and sets the state to Idle.
  //
  // This is called from an ISR.
  void completePacket();

  // These are only filled in if this Sender uses an LPUART
  LPUARTParams lpuartBreakParams_;
  LPUARTParams lpuartSlotsParams_;
  bool lpuartParamsSet_;

  // Tracks whether the system has been configured.
  volatile bool began_;

  // Keeps track of what we're transmitting.
  volatile XmitStates state_;

  volatile uint8_t outputBuf_[kMaxDMXPacketSize];
  int outputBufIndex_;

  // BREAK and MAB timings
  volatile uint32_t breakTime_;
  uint32_t mabTime_;
  volatile uint32_t adjustedMABTime_;  // Adjusted for the real world; requested

  // The size of the packet to be sent.
  volatile int packetSize_;

  // The packet refresh rate, in Hz.
  float refreshRate_;
  util::IntervalTimer intervalTimer_;  // General purpose timer

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

  // This is called when we are done transmitting after a resumeFor call.
  void (*volatile doneTXFunc_)(Sender *s);

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

#if defined(IMXRT_LPUART5) && defined(__IMXRT1052__)
  friend void lpuart5_tx_isr();
#endif  // IMXRT_LPUART5 && __IMXRT1052__
};

}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_TEENSYDMX_H_
