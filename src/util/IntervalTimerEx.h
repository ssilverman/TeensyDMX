// IntervalTimerEx.h defines an interface to the Periodic Interrupt Timers by
// extending the Teensy library's IntervalTimer with proper callbacks.
// This file is part of the TeensyDMX library.
// (c) 2020-2022 Shawn Silverman

// See also https://github.com/luni64/TeensyHelpers for luni64's take on
// improved IntervalTimer usage; it was the basis for this code.

#ifndef TEENSYDMX_USE_PERIODICTIMER

#ifndef TEENSYDMX_UTIL_INTERVALTIMEREX_H_
#define TEENSYDMX_UTIL_INTERVALTIMEREX_H_

// C++ includes
#include <functional>

#include <IntervalTimer.h>
#include <util/atomic.h>

namespace qindesign {
namespace teensydmx {
namespace util {

#if defined(KINETISK)
static constexpr int kNumChannels = 4;
#elif defined(KINETISL)
static constexpr int kNumChannels = 2;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
static constexpr int kNumChannels = 4;
#endif  // Processor check

// Wraps IntervalTimer so that we can use function callbacks.
class IntervalTimerEx final {
 public:
  IntervalTimerEx()
      : intervalTimer_{},
        cbIndex_(0),
        started_(false) {}

  ~IntervalTimerEx();

  // Attempts to start or restart a timer. This returns whether the timer was
  // successfully started or restarted. This version of the function replaces
  // the callback if the timer was already started.
  template <typename period_t>
  bool begin(const std::function<void()> &callback, period_t period) {
    // Find a free slot, if not already started
    if (!started_) {
      for (int i = 0; i < kNumChannels; i++) {
        if (callbacks_[i] != nullptr) {
          continue;
        }

        // Make begin and setting the callback atomic
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          if (intervalTimer_.begin(relays_[i], period)) {
            callbacks_[i] = callback;
            cbIndex_ = i;
            started_ = true;
            return true;
          }
        }
      }
    } else {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (intervalTimer_.begin(relays_[cbIndex_], period)) {
          callbacks_[cbIndex_] = callback;
          return true;
        }
      }
      end();
    }

    return false;
  }

  // Restarts the timer using the current callback. This returns whether the
  // timer was successfully restarted. This returns false if the timer is not
  // already started.
  template <typename period_t>
  bool restart(period_t period) {
    if (!started_) {
      return false;
    }
    if (!intervalTimer_.begin(relays_[cbIndex_], period)) {
      end();
      return false;
    }
    return true;
  }

  // Sets the timer priority, if running.
  void setPriority(uint8_t n);

  // Stops the current timer, if running.
  void end();

 private:
  static std::function<void()> callbacks_[kNumChannels];
  static void (*relays_[kNumChannels])(void);

  // Use composition rather than inheritance to avoid the whole destructor mess
  IntervalTimer intervalTimer_;

  int cbIndex_;   // The callback index
  bool started_;  // Whether a timer has been started
};

}  // namespace util
}  // namespace teensydmx
}  // namespace qindesign

#endif  // TEENSYDMX_UTIL_INTERVALTIMEREX_H_

#endif  // !TEENSYDMX_USE_PERIODICTIMER
