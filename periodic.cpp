#include "periodic.h"
#include <time.h>
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>

std::atomic<bool> running(true);
std::atomic<long> current_period_ns(0); // 0 = not set yet

void sigint_handler(int)
{
    running = false;
}

static void sleep_until_time(const struct timespec &next)
{
#if defined(__APPLE__)
    // macOS: no clock_nanosleep, use std::chrono
    using namespace std::chrono;
    auto tp = steady_clock::now() + seconds(next.tv_sec - time(nullptr)) + nanoseconds(next.tv_nsec);
    std::this_thread::sleep_until(tp);
#elif defined(__linux__)
    // Linux: use clock_nanosleep with TIMER_ABSTIME
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
#else
    // Fallback for other platforms
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    long sec = next.tv_sec - now.tv_sec;
    long nsec = next.tv_nsec - now.tv_nsec;
    if (nsec < 0)
    {
        sec--;
        nsec += 1000000000;
    }
    if (sec > 0 || nsec > 0)
    {
        struct timespec req = {sec, nsec};
        nanosleep(&req, nullptr);
    }
#endif
}

void run_periodic(void (*func)())
{
    signal(SIGINT, sigint_handler);

    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);

    while (running.load())
    {
        // wait until a valid period is set
        long period_ns = current_period_ns.load();
        if (period_ns <= 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        func();

        // update next wake-up time
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000)
        {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }

        sleep_until_time(next);
    }
}

void update_period(long new_period_ns)
{
    if (new_period_ns > 0)
        current_period_ns.store(new_period_ns);
}
