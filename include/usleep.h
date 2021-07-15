#pragma once
#include <thread>
#include <chrono>

static inline void usleep(unsigned int a_microsec) {
    std::this_thread::sleep_for(std::chrono::microseconds(a_microsec));
}