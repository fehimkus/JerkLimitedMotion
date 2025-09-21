#include "periodic.h"
#include <time.h>
#include <csignal>
#include <thread>

std::atomic<bool> running(true);
std::atomic<long> current_period_ns(0); // 0 = not set yet

void sigint_handler(int)
{
    running = false;
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

        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000)
        {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
}

void update_period(long new_period_ns)
{
    if (new_period_ns > 0)
        current_period_ns.store(new_period_ns);
}
