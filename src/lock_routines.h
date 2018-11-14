// Routines for concurrency.

#include <Arduino.h>

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
