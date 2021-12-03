// PeriodicTimer.cpp implements PeriodicTimer.
// This file is part of the TeensyDMX library.
// (c) 2019-2021 Shawn Silverman

#ifndef USE_INTERVALTIMER

#include "PeriodicTimer.h"

// C++ includes
#include <algorithm>
#include <functional>

#include <Arduino.h>

// Global variables:
// * Running states
// * Functions
// * Function states
// * Priorities

// This is defined so that calling a std::function<void()> can compile
// when size optimization is enabled. Teensy LC has "Smallest Code"
// set by default, for example.
namespace std {
  void __throw_bad_function_call() __attribute__((weak));
  [[noreturn]] void __throw_bad_function_call() {
    Serial.println("EXCEPTION: Bad function call!");
    while (true) {
      // Don't return
      yield();
    }
  }
}  // namespace std

#if defined(KINETISK)
extern "C" {
extern void unused_isr(void);

static void (*oldISRs[4])(void){unused_isr, unused_isr, unused_isr, unused_isr};
static uint8_t oldPriorities[4]{128, 128, 128, 128};
}
#elif defined(KINETISL)
extern "C" {
extern void unused_isr(void);

static void (*oldISR)(void) = unused_isr;
static uint8_t oldPriority = 128;
}
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
// For handling an old interrupt vector
extern "C" {
extern void unused_interrupt_vector(void);

static void (*oldISR)(void) = unused_interrupt_vector;
static uint8_t oldPriority = 128;
}
#endif  // Which chip?

namespace qindesign {
namespace teensydmx {
namespace util {

// Note: When we use a std::function (Null Pattern) instead of
//       nullptr, replacing the function during end(), when called in
//       the lambda seems to also replace the local 'this' pointer to
//       the containing code, if the end() call isn't the last call in
//       the lambda. Perhaps the 'operator=' operation does something
//       funky? Is this expected or is it a compiler issue?
//       Changing the API to use 'const std::function' doesn't seem
//       to help.
// static const std::function<void()> nullFunc = []() {};

#if defined(KINETISK)
static void my_pit0_isr();
static void my_pit1_isr();
static void my_pit2_isr();
static void my_pit3_isr();
static constexpr int kNumChannels = 4;
static std::function<void()> funcs[kNumChannels]{
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};
#elif defined(KINETISL)
static void my_pit_isr();
static constexpr int kNumChannels = 2;
static uint32_t runningFlags = 0;
static std::function<void()> funcs[kNumChannels]{
    nullptr,
    nullptr,
};
static uint8_t priorities[kNumChannels]{255, 255};
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
static void pit_isr();
static constexpr int kNumChannels = 4;
static uint32_t runningFlags = 0;
static std::function<void()> funcs[kNumChannels] __attribute((aligned(32))){
    nullptr,
    nullptr,
    nullptr,
    nullptr,
};
static uint8_t priorities[kNumChannels]{255, 255, 255, 255};
#endif  // Processor check

#if defined(KINETISK) || defined(KINETISL)
static constexpr uint32_t kFreq = F_BUS;
// Maximum period in microseconds.
static constexpr uint32_t kMaxPeriod = UINT32_MAX / (kFreq / 1000000.0);
static constexpr uint32_t kMinCycles = 36;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
static constexpr uint32_t kFreq = 24000000;
// Maximum period in microseconds.
static constexpr uint32_t kMaxPeriod = UINT32_MAX / (kFreq / 1000000);
static constexpr uint32_t kMinCycles = 17;
#endif  // Processor check

// Checks uint32_t micros.
static bool checkMicros(uint32_t micros) {
  return (0 != micros) && (micros <= kMaxPeriod);
}

// Checks float micros. Comparing to zero is sufficient because
// -0.5 will truncate to zero when converting to a uint32_t.
static bool checkMicros(float micros) {
  return (0.0f <= micros) && (micros <= kMaxPeriod);
}

bool PeriodicTimer::begin(std::function<void()> func, uint32_t micros,
                          std::function<void()> startFunc) {
  if (!checkMicros(micros)) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000)*micros - 1;
  return beginCycles(func, cycles, startFunc);
}

bool PeriodicTimer::begin(std::function<void()> func, float micros,
                          std::function<void()> startFunc) {
  if (!checkMicros(micros)) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000.0f)*micros - 0.5f;
  return beginCycles(func, cycles, startFunc);
}

bool PeriodicTimer::restart(uint32_t micros) {
  if (!checkMicros(micros)) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000)*micros - 1;
  return restartCycles(cycles);
}

bool PeriodicTimer::restart(float micros) {
  if (!checkMicros(micros)) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000.0f)*micros - 0.5f;
  return restartCycles(cycles);
}

bool PeriodicTimer::update(uint32_t micros) {
  if (!checkMicros(micros)) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000)*micros - 1;
  return updateCycles(cycles);
}

bool PeriodicTimer::update(float micros) {
  if (!checkMicros(micros)) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000.0f)*micros - 0.5f;
  return updateCycles(cycles);
}

bool PeriodicTimer::updateCycles(uint32_t cycles) {
  // Lock lock{};
  if (channel_ == nullptr) {
    return false;
  }
  if (cycles < kMinCycles) {
    return false;
  }
  channel_->LDVAL = cycles;
  return true;
}

bool PeriodicTimer::restartCycles(uint32_t cycles) {
  // Lock lock{};
  if (channel_ == nullptr) {
    return false;
  }
  if (cycles < kMinCycles) {
    return false;
  }
  channel_->TCTRL = 0;            // Disable the timer so it can be restarted
  channel_->TFLG = PIT_TFLG_TIF;  // Clear the interrupt
  channel_->LDVAL = cycles;
  channel_->TCTRL = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
  return true;
}

bool PeriodicTimer::beginCycles(std::function<void()> func, uint32_t cycles,
                                std::function<void()> startFunc) {
  // Lock lock{};

  if (cycles < kMinCycles) {
    return false;
  }

  // Capture the timer
  if (channel_ != nullptr) {
    channel_->TCTRL = 0;            // Disable the timer so it can be restarted
    channel_->TFLG = PIT_TFLG_TIF;  // Clear the interrupt
  } else {
#if defined(KINETISK) || defined(KINETISL)
    SIM_SCGC6 |= SIM_SCGC6_PIT;
    __asm__ volatile("nop");  // Solves some timing problem on Teensy 3.5,
                              // per PJRC's implementation
    PIT_MCR = PIT_MCR_FRZ;  // Allow freeze in debug mode
    KINETISK_PIT_CHANNEL_t *ch = KINETISK_PIT_CHANNELS;
    while (true) {
      if (ch->TCTRL == 0) {
        break;
      }
      if (++ch >= KINETISK_PIT_CHANNELS + kNumChannels) {
        return false;
      }
    }
    channel_ = ch;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
    CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
    PIT_MCR = PIT_MCR_FRZ;  // Allow freeze in debug mode
    IMXRT_PIT_CHANNEL_t *ch = IMXRT_PIT_CHANNELS;
    while (true) {
      if (ch->TCTRL == 0) {
        break;
      }
      if (++ch >= IMXRT_PIT_CHANNELS + kNumChannels) {
        return false;
      }
    }
    channel_ = ch;
#endif  // Processor check
  }

  // Start the timer
#if defined(KINETISK)
  int index = channel_ - KINETISK_PIT_CHANNELS;
#elif defined(KINETISL)
  int index = channel_ - KINETISK_PIT_CHANNELS;
  runningFlags |= (uint32_t{1} << index);
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
  int index = channel_ - IMXRT_PIT_CHANNELS;
  runningFlags |= (uint32_t{1} << index);
#endif  // Processor check
  funcs[index] = func;
  channel_->LDVAL = cycles;
  channel_->TCTRL = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
  if (startFunc != nullptr) {
    startFunc();
  }
#if defined(KINETISK)
  oldISRs[index] = _VectorsRam[IRQ_PIT_CH0 + index + 16];
  oldPriorities[index] = NVIC_GET_PRIORITY(IRQ_PIT_CH0 + index);
  switch (index) {
    case 0:
      attachInterruptVector(IRQ_PIT_CH0, &my_pit0_isr);
      break;
    case 1:
      attachInterruptVector(IRQ_PIT_CH1, &my_pit1_isr);
      break;
    case 2:
      attachInterruptVector(IRQ_PIT_CH2, &my_pit2_isr);
      break;
    case 3:
      attachInterruptVector(IRQ_PIT_CH3, &my_pit3_isr);
      break;
  }
  NVIC_SET_PRIORITY(IRQ_PIT_CH0 + index, priority_);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0 + index);
#elif defined(KINETISL)
  priorities[index] = priority_;
  oldISR = _VectorsRam[IRQ_PIT + 16];
  oldPriority = NVIC_GET_PRIORITY(IRQ_PIT);
  attachInterruptVector(IRQ_PIT, &my_pit_isr);
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
  NVIC_ENABLE_IRQ(IRQ_PIT);
#elif defined(__IMXRT1062__) || (__IMXRT1052__)
  priorities[index] = priority_;
  oldISR = _VectorsRam[IRQ_PIT + 16];
  oldPriority = NVIC_GET_PRIORITY(IRQ_PIT);
  attachInterruptVector(IRQ_PIT, &pit_isr);
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
  NVIC_ENABLE_IRQ(IRQ_PIT);
#endif  // Processor check
  return true;
}

void PeriodicTimer::end() {
  // Lock lock{};

  if (channel_ == nullptr) {
    return;
  }
  channel_->TCTRL = 0;

#if defined(KINETISK)
  int index = channel_ - KINETISK_PIT_CHANNELS;
  funcs[index] = nullptr;
  if (oldISRs[index] == unused_isr) {
    NVIC_DISABLE_IRQ(IRQ_PIT_CH0 + index);
  }
  attachInterruptVector(static_cast<IRQ_NUMBER_t>(IRQ_PIT_CH0 + index),
                        oldISRs[index]);
  NVIC_SET_PRIORITY(IRQ_PIT_CH0 + index, oldPriorities[index]);
#elif defined(KINETISL)
  int index = channel_ - KINETISK_PIT_CHANNELS;
  funcs[index] = nullptr;
  priorities[index] = 255;
  runningFlags &= ~(uint32_t{1} << index);
  if (runningFlags == 0) {
    if (oldISR == unused_isr) {
      NVIC_DISABLE_IRQ(IRQ_PIT);
    }
    attachInterruptVector(IRQ_PIT, oldISR);
    NVIC_SET_PRIORITY(IRQ_PIT, oldPriority);
  } else {
    NVIC_SET_PRIORITY(IRQ_PIT,
                      *std::min_element(&priorities[0],
                                        &priorities[kNumChannels]));
  }
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
  int index = channel_ - IMXRT_PIT_CHANNELS;
  funcs[index] = nullptr;
  priorities[index] = 255;
  runningFlags &= ~(uint32_t{1} << index);
  if (runningFlags == 0) {
    if (oldISR == unused_interrupt_vector) {
      NVIC_DISABLE_IRQ(IRQ_PIT);
    }
    attachInterruptVector(IRQ_PIT, oldISR);
    NVIC_SET_PRIORITY(IRQ_PIT, oldPriority);
  } else {
    NVIC_SET_PRIORITY(IRQ_PIT,
                      *std::min_element(&priorities[0],
                                        &priorities[kNumChannels]));
  }
#endif  // Processor check
  channel_ = nullptr;
}

void PeriodicTimer::setPriority(uint8_t n) {
  // Lock lock{};

  priority_ = n;
  if (channel_ == nullptr) {
    return;
  }

#if defined(KINETISK)
  int index = channel_ - KINETISK_PIT_CHANNELS;
  NVIC_SET_PRIORITY(IRQ_PIT_CH0 + index, n);
#elif defined(KINETISL)
  int index = channel_ - KINETISK_PIT_CHANNELS;
  priorities[index] = n;
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
  int index = channel_ - IMXRT_PIT_CHANNELS;
  priorities[index] = n;
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
#endif  // Processor check
}

// ---------------------------------------------------------------------------
//  ISRs
// ---------------------------------------------------------------------------

#if defined(KINETISK)
static void my_pit0_isr() {
  if (funcs[0] != nullptr) {
    funcs[0]();
  }
  if (oldISRs[0] != unused_isr) {
    oldISRs[0]();
  }

  // Clear the interrupt after calling any old ISR
  PIT_TFLG0 = 1;
}

static void my_pit1_isr() {
  if (funcs[1] != nullptr) {
    funcs[1]();
  }
  if (oldISRs[1] != unused_isr) {
    oldISRs[1]();
  }

  // Clear the interrupt after calling any old ISR
  PIT_TFLG1 = 1;
}

static void my_pit2_isr() {
  if (funcs[2] != nullptr) {
    funcs[2]();
  }
  if (oldISRs[2] != unused_isr) {
    oldISRs[2]();
  }

  // Clear the interrupt after calling any old ISR
  PIT_TFLG2 = 1;
}

static void my_pit3_isr() {
  if (funcs[3] != nullptr) {
    funcs[3]();
  }
  if (oldISRs[3] != unused_isr) {
    oldISRs[3]();
  }

  // Clear the interrupt after calling any old ISR
  PIT_TFLG3 = 1;
}
#elif defined(KINETISL)
static void my_pit_isr() {
  if (PIT_TFLG0 != 0) {
    if (funcs[0] != nullptr) {
      funcs[0]();
    }
  }
  if (PIT_TFLG1 != 0) {
    if (funcs[1] != nullptr) {
      funcs[1]();
    }
  }
  if (oldISR != unused_isr) {
    oldISR();
  }

  // Clear the interrupts after calling any old ISR
  PIT_TFLG0 = 1;
  PIT_TFLG1 = 1;
}
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
static void pit_isr() {
  if (PIT_TFLG0 != 0) {
    if (funcs[0] != nullptr) {
      funcs[0]();
    }
  }
  if (PIT_TFLG1 != 0) {
    if (funcs[1] != nullptr) {
      funcs[1]();
    }
  }
  if (PIT_TFLG2 != 0) {
    if (funcs[2] != nullptr) {
      funcs[2]();
    }
  }
  if (PIT_TFLG3 != 0) {
    if (funcs[3] != nullptr) {
      funcs[3]();
    }
  }
  if (oldISR != unused_interrupt_vector) {
    oldISR();
  }

  // Clear the interrupts after calling any old ISR
  PIT_TFLG0 = 1;
  PIT_TFLG1 = 1;
  PIT_TFLG2 = 1;
  PIT_TFLG3 = 1;
  asm("dsb");
}
#endif  // Processor check

}  // namespace util
}  // namespace teensydmx
}  // namespace qindesign

#endif  // !USE_INTERVALTIMER
