// Routines for concurrency.

#ifndef LOCK_ROUTINES_H_
#define LOCK_ROUTINES_H_

// C++ includes
#include <atomic>

// Other includes
#include <Arduino.h>

namespace qindesign {
namespace teensydmx {

// Implements a simple spinlock mutex using std::atomic_flag.
class Mutex final {
 public:
  Mutex() {}

  // Grabs the mutex.
  void acquire() {
    while (m_.test_and_set(std::memory_order_acquire)) {
      yield();
    }
  }

  // Releases the mutex. This assumes that whoever is calling this owns
  // the mutex.
  void release() {
    m_.clear(std::memory_order_release);
  }

 private:
  std::atomic_flag m_;
};

// Grabs a mutex. This isn't reentrant, nor is it safe across function calls.
static void grabMutex(volatile bool *m) {
  __disable_irq();
  while (*m) {
    __enable_irq();
    yield();
    __disable_irq();
  }
  *m = true;
  __enable_irq();
}

// Releases a mutex. This assumes that whoever is calling this owns the mutex.
static void releaseMutex(volatile bool *m) {
  __disable_irq();
  *m = false;
  __enable_irq();
}

}  // namespace teensydmx
}  // namespace qindesign

#endif  // LOCK_ROUTINES_H_
