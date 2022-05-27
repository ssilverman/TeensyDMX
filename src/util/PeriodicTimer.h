// PeriodicTimer.h defines an interface to the Periodic Interrupt
// Timers (PIT). It has a flexible API and the ability to use lambdas
// for the trigger function. It is similar to PJRC's IntervalTimer
// from Teensyduino.
// This file is part of the TeensyDMX library.
// (c) 2019-2021 Shawn Silverman

#ifdef TEENSYDMX_USE_PERIODICTIMER

#ifndef TEENSYDMX_UTIL_PERIODICTIMER_H_
#define TEENSYDMX_UTIL_PERIODICTIMER_H_

// C++ includes
#include <cstdint>
#include <functional>

#if defined(__MK20DX128__) || defined(__MK20DX256__) || \
    defined(__MKL26Z64__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
#include <kinetis.h>
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
#include <imxrt.h>
#endif  // Processor check

namespace qindesign {
namespace teensydmx {
namespace util {

// Defines a timer that triggers a function periodically. Under the
// covers, this uses the Periodic Interrupt Timers (PIT). There are a
// limited number of them, so they are first come first served.
//
// This class is not safe in the presence of concurrency.
class PeriodicTimer final {
 public:
  constexpr PeriodicTimer()
      : channel_(nullptr),
        priority_(128) {}

  ~PeriodicTimer() {
    end();
  }

  // Starts the periodic timer for the specified number of
  // microseconds. This returns true if a timer was found and false
  // otherwise. This also returns false if the period is zero or out
  // of range.
  //
  // This sets up the function and enables the interrupt.
  //
  // The `startFunc` argument is optional and can be specified if a
  // function needs to be called just before the timer interrupt is
  // enabled. This helps to make timing more accurate by moving
  // statements that need to be executed at the start of the interval
  // to just before the interval actually begins.
  bool begin(std::function<void()> func, uint32_t micros,
             std::function<void()> startFunc = nullptr);

  // See the `uint32_t` version of `begin`. This also returns false if
  // the period is negative.
  bool begin(std::function<void()> func, float micros,
             std::function<void()> startFunc = nullptr);

  // Restarts the timer with the specified period in microseconds. The
  // timer is first stopped and then restarted with the new period.
  //
  // This returns false if the timer is not set up, or if the period
  // is zero or out of range. Otherwise, this returns true.
  bool restart(uint32_t micros);

  // See the `uint32_t` version of `restart`. This will also return
  // false if the period is negative.
  bool restart(float micros);

  // Immediately updates the timer with the specified period in
  // microseconds. The new period will start as soon as the current
  // one finishes.
  //
  // This returns false if the timer is not set up, or if the period
  // is zero or out of range. Otherwise, this returns true.
  bool update(uint32_t micros);

  // See the `uint32_t` version of `update`. This also returns false
  // if the period is negative.
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
  // // This is used because it's possible to mess around with these
  // // objects from inside interrupts.
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
  KINETISK_PIT_CHANNEL_t *volatile channel_;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
  IMXRT_PIT_CHANNEL_t *volatile channel_;
#endif  // Processor check
  volatile uint8_t priority_;

  bool beginCycles(std::function<void()> func, uint32_t cycles,
                   std::function<void()> startFunc);
  bool updateCycles(uint32_t cycles);
  bool restartCycles(uint32_t cycles);
};

}  // namespace util
}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_UTIL_PERIODICTIMER_H_

#endif  // TEENSYDMX_USE_PERIODICTIMER
