// IntervalTimer.h defines an enhanced version of PJRC's Teensyduino
// IntervalTimer that accepts state as arguments to the trigger functions.
// This file is part of the TeensyDMX library.
// (c) 2019 Shawn Silverman

#ifndef TEENSYDMX_UTIL_INTERVALTIMER_H_
#define TEENSYDMX_UTIL_INTERVALTIMER_H_

// C++ includes
#include <cstdint>

#if defined(__MK20DX128__) || defined(__MK20DX256__) || \
    defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#include <kinetis.h>
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
#include <imxrt.h>
#endif  // Processor check

namespace qindesign {
namespace util {

// Defines a timer that triggers a function periodically. Under the covers, this
// uses the Periodic Interrupt Timers (PIT). There are a limited number of them,
// so they are first come first served.
//
// This class is not safe in the presence of concurrency.
class IntervalTimer final {
 public:
  constexpr IntervalTimer()
      : channel_(nullptr),
        priority_(128) {}

  ~IntervalTimer() {
    end();
  }

  // Starts the periodic timer for the specified number of microseconds. This
  // returns true if a timer was found and false otherwise. This also returns
  // false if the period is zero or out of range.
  //
  // This sets up the function and enables the interrupt.
  //
  // The `startFunc` and `startState` arguments are optional and can be
  // specified if a function needs to be called just before the timer interrupt
  // is enabled. This helps to make timing more accurate by moving statements
  // that need to be executed at the start of the interval to just before the
  // interval actually begins.
  bool begin(void (*func)(void *), void *state, uint32_t micros,
             void (*startFunc)(void *) = nullptr, void *startState = nullptr);

  // See the `uint32_t` version of `begin`. This also returns false if the
  // period is negative.
  bool begin(void (*func)(void *), void *state, float micros,
             void (*startFunc)(void *) = nullptr, void *startState = nullptr);

  // Restarts the timer with the specified period in microseconds. The timer is
  // first stopped and then restarted with the new period.
  //
  // This returns false if the timer is not set up, or if the period is zero or
  // out of range. Otherwise, this returns true.
  bool restart(uint32_t micros);

  // See the `uint32_t` version of `restart`. This will also return false if the
  // period is negative.
  bool restart(float micros);

  // Immediately updates the timer with the specified period in microseconds.
  // The new period will start as soon as the current one finishes.
  //
  // This returns false if the timer is not set up, or if the period is zero or
  // out of range. Otherwise, this returns true.
  bool update(uint32_t micros);

  // See the `uint32_t` version of `update`. This also returns false if the
  // period is negative.
  bool update(float micros);

  // Disables the timer and releases any resources.
  void end();

  void setPriority(uint8_t n);

  uint8_t priority() const {
    return priority_;
  }

 private:
  // // Use RAII to disable and re-enable interrupts.
  // //
  // // This is used because it's possible to mess around with IntervalTimers from
  // // inside interrupts.
  // class Lock final {
  //  public:
  //   Lock() {
  //     __disable_irq();
  //   }
  //   ~Lock() {
  //     __enable_irq();
  //   }
  // };

#if defined(KINETISK) || defined(KINETISL)
	KINETISK_PIT_CHANNEL_t */*volatile*/ channel_;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
	IMXRT_PIT_CHANNEL_t */*volatile*/ channel_;
#endif  // Processor check
	/*volatile*/ uint8_t priority_;

	bool beginCycles(void (*func)(void *), void *state, uint32_t cycles,
                   void (*startFunc)(void *), void *startState);
  bool updateCycles(uint32_t cycles);
  bool restartCycles(uint32_t cycles);
};

}  // namespace util
}  // namespace qindesign

#endif  // TEENSYDMX_UTIL_INTERVALTIMER_H_
