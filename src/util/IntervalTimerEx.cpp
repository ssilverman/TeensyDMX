// IntervalTimerEx.cpp implements IntervalTimerEx.
// This file is part of the TeensyDMX library.
// (c) 2020-2021 Shawn Silverman

#ifndef TEENSYDMX_USE_PERIODICTIMER

#include "IntervalTimerEx.h"

namespace qindesign {
namespace teensydmx {
namespace util {

std::function<void()> IntervalTimerEx::callbacks_[kNumChannels]{nullptr};

#ifdef KINETISL
void (*IntervalTimerEx::relays_[2])(void){
    []() { if (callbacks_[0] != nullptr) { callbacks_[0](); } },
    []() { if (callbacks_[1] != nullptr) { callbacks_[1](); } },
};
#else
void (*IntervalTimerEx::relays_[4])(void){
    []() { if (callbacks_[0] != nullptr) { callbacks_[0](); } },
    []() { if (callbacks_[1] != nullptr) { callbacks_[1](); } },
    []() { if (callbacks_[2] != nullptr) { callbacks_[2](); } },
    []() { if (callbacks_[3] != nullptr) { callbacks_[3](); } },
};
#endif  // KINETISL

IntervalTimerEx::~IntervalTimerEx() {
  end();
}

void IntervalTimerEx::setPriority(uint8_t n) {
  if (started_) {
    intervalTimer_.priority(n);
  }
}

void IntervalTimerEx::end() {
  if (started_) {
    started_ = false;
    intervalTimer_.end();  // Call this before removing the callback
    callbacks_[cbIndex_] = nullptr;
  }
}

}  // namespace util
}  // namespace teensydmx
}  // namespace qindesign

#endif  // !TEENSYDMX_USE_PERIODICTIMER
