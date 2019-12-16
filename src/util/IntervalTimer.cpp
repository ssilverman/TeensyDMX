// IntervalTimer.cpp implements the enhanced IntervalTimer.
// This file is part of the TeensyDMX library.
// (c) 2019 Shawn Silverman

#include "IntervalTimer.h"

// C++ includes
#include <algorithm>

#include <Arduino.h>

// Global variables:
// * Running states
// * Functions
// * Function states
// * Priorities

namespace qindesign {
namespace util {

#if defined(KINETISK)
static void pit0_isr();
static void pit1_isr();
static void pit2_isr();
static void pit3_isr();
static constexpr int kNumChannels = 4;
static void (*funcs[kNumChannels])(void *){nullptr};
static void *funcStates[kNumChannels]{nullptr};
#elif defined(KINETISL)
static void pit_isr();
static constexpr int kNumChannels = 2;
static uint32_t runningFlags = 0;
static void (*funcs[kNumChannels])(void *){nullptr};
static void *funcStates[kNumChannels]{nullptr};
static uint8_t priorities[kNumChannels]{255, 255};
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
static void pit_isr();
static constexpr int kNumChannels = 4;
static uint32_t runningFlags = 0;
static void (*funcs[kNumChannels])(void *) __attribute((aligned(32))){nullptr};
static void *funcStates[kNumChannels] __attribute((aligned(32))){nullptr};
static uint8_t priorities[kNumChannels]{255, 255, 255, 255};
#endif  // Processor check

#if defined(KINETISK) || defined(KINETISL)
static constexpr uint32_t kFreq = F_BUS;
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
static constexpr uint32_t kFreq = 24000000;
#endif  // Processor check

// Maximum period in microseconds.
static constexpr uint32_t kMaxPeriod = UINT32_MAX / (kFreq / 1000000.0f);

bool IntervalTimer::begin(void (*func)(void *), void *state, uint32_t micros) {
  if (micros == 0 || kMaxPeriod < micros) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000)*micros - 1;
  return beginCycles(func, state, cycles);
}

bool IntervalTimer::begin(void (*func)(void *), void *state, float micros) {
  if (micros <= 0 || kMaxPeriod < micros) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000.0f)*micros - 0.5f;
  return beginCycles(func, state, cycles);
}

bool IntervalTimer::restart(uint32_t micros) {
  if (micros == 0 || kMaxPeriod < micros) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000)*micros - 1;
  return restartCycles(cycles);
}

bool IntervalTimer::restart(float micros) {
  if (micros <= 0 || kMaxPeriod < micros) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000.0f)*micros - 0.5f;
  return restartCycles(cycles);
}

bool IntervalTimer::update(uint32_t micros) {
  if (micros == 0 || kMaxPeriod < micros) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000)*micros - 1;
  return updateCycles(cycles);
}

bool IntervalTimer::update(float micros) {
  if (micros <= 0 || kMaxPeriod < micros) {
    return false;
  }
  uint32_t cycles = (kFreq/1000000.0f)*micros - 0.5f;
  return updateCycles(cycles);
}

bool IntervalTimer::updateCycles(uint32_t cycles) {
  if (cycles < 36) {  // TODO: Why 36?
    return false;
  }

  // Lock lock{};
  if (channel_ == nullptr) {
    return false;
  }
  channel_->LDVAL = cycles;
  return true;
}

bool IntervalTimer::restartCycles(uint32_t cycles) {
  if (cycles < 36) {  // TODO: Why 36?
    return false;
  }

  // Lock lock{};
  if (channel_ == nullptr) {
    return false;
  }
  channel_->TCTRL = 0;            // Disable the timer so it can be restarted
  channel_->TFLG = PIT_TFLG_TIF;  // Clear the interrupt
  channel_->LDVAL = cycles;
  channel_->TCTRL = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
  return true;
}

bool IntervalTimer::beginCycles(void (*func)(void *), void *state,
                                uint32_t cycles) {
  if (cycles < 36) {  // TODO: Why 36?
    return false;
  }

  // Lock lock{};

  // Capture the timer
  if (channel_ != nullptr) {
    channel_->TCTRL = 0;            // Disable the timer so it can be restarted
    channel_->TFLG = PIT_TFLG_TIF;  // Clear the interrupt
	} else {
#if defined(KINETISK) || defined(KINETISL)
		SIM_SCGC6 |= SIM_SCGC6_PIT;
		__asm__ volatile("nop");  // Solves timing problem on Teensy 3.5
		PIT_MCR = PIT_MCR_FRZ;
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
		PIT_MCR = PIT_MCR_FRZ;
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
  funcStates[index] = state;
  channel_->LDVAL = cycles;
  channel_->TCTRL = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
#if defined(KINETISK)
  switch (index) {
    case 0:
      attachInterruptVector(IRQ_PIT_CH0, &pit0_isr);
      break;
    case 1:
      attachInterruptVector(IRQ_PIT_CH1, &pit1_isr);
      break;
    case 2:
      attachInterruptVector(IRQ_PIT_CH2, &pit2_isr);
      break;
    case 3:
      attachInterruptVector(IRQ_PIT_CH3, &pit3_isr);
      break;
  }
  NVIC_SET_PRIORITY(IRQ_PIT_CH0 + index, priority_);
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0 + index);
#elif defined(KINETISL)
	priorities[index] = priority_;
  attachInterruptVector(IRQ_PIT, &pit_isr);
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
	NVIC_ENABLE_IRQ(IRQ_PIT);
#elif defined(__IMXRT1062__) || (__IMXRT1052__)
	priorities[index] = priority_;
  attachInterruptVector(IRQ_PIT, &pit_isr);
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
	NVIC_ENABLE_IRQ(IRQ_PIT);
#endif  // Processor check
  return true;
}

void IntervalTimer::end() {
  // Lock lock{};

  if (channel_ == nullptr) {
    return;
  }

#if defined(KINETISK)
  int index = channel_ - KINETISK_PIT_CHANNELS;
  NVIC_DISABLE_IRQ(IRQ_PIT_CH0 + index);
#elif defined(KINETISL)
  int index = channel_ - KINETISK_PIT_CHANNELS;
  runningFlags &= ~(uint32_t{1} << index);
  if (runningFlags == 0) {
    NVIC_DISABLE_IRQ(IRQ_PIT);
  }
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
  int index = channel_ - IMXRT_PIT_CHANNELS;
  runningFlags &= ~(uint32_t{1} << index);
  if (runningFlags == 0) {
    NVIC_DISABLE_IRQ(IRQ_PIT);
  }
#endif  // Processor check
  funcs[index] = nullptr;
  channel_->TCTRL = 0;
#if defined(KINETISL)
  priorities[index] = 255;
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
  priorities[index] = 255;
  NVIC_SET_PRIORITY(IRQ_PIT,
                    *std::min_element(&priorities[0],
                                      &priorities[kNumChannels]));
#endif  // Processor check
  channel_ = nullptr;
}

void IntervalTimer::setPriority(uint8_t n) {
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
void pit0_isr() {
	PIT_TFLG0 = 1;
  if (funcs[0] != nullptr) {
  	funcs[0](funcStates[0]);
  }
}

void pit1_isr() {
  PIT_TFLG1 = 1;
  if (funcs[1] != nullptr) {
    funcs[1](funcStates[1]);
  }
}

void pit2_isr() {
  PIT_TFLG2 = 1;
  if (funcs[2] != nullptr) {
    funcs[2](funcStates[2]);
  }
}

void pit3_isr() {
  PIT_TFLG3 = 1;
  if (funcs[3] != nullptr) {
    funcs[3](funcStates[3]);
  }
}
#elif defined(KINETISL)
void pit_isr() {
  if (PIT_TFLG0 != 0) {
    PIT_TFLG0 = 1;
    if (funcs[0] != nullptr) {
      funcs[0](funcStates[0]);
    }
	}
	if (PIT_TFLG1 != 0) {
		PIT_TFLG1 = 1;
    if (funcs[1] != nullptr) {
      funcs[1](funcStates[1]);
    }
	}
}
#elif defined(__IMXRT1062__) || defined(__IMXRT1052__)
void pit_isr() {
  if (PIT_TFLG0 != 0) {
    PIT_TFLG0 = 1;
    if (funcs[0] != nullptr) {
      funcs[0](funcStates[0]);
    }
  }
  if (PIT_TFLG1 != 0) {
    PIT_TFLG1 = 1;
    if (funcs[1] != nullptr) {
      funcs[1](funcStates[1]);
    }
  }
  if (PIT_TFLG2 != 0) {
    PIT_TFLG2 = 1;
    if (funcs[2] != nullptr) {
      funcs[2](funcStates[2]);
    }
  }
  if (PIT_TFLG3 != 0) {
    PIT_TFLG3 = 1;
    if (funcs[3] != nullptr) {
      funcs[3](funcStates[3]);
    }
  }
}
#endif  // Processor check

}  // namespace util
}  // namespace qindesign
