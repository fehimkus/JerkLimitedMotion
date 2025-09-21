#pragma once
#include <atomic>

// Globals
extern std::atomic<bool> running;
extern std::atomic<long> current_period_ns;

// Function to start periodic loop
void run_periodic(void (*func)());

// Function to update the period while running
void update_period(long new_period_ns);
