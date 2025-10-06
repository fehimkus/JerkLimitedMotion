#include "periodic.h"
#include <time.h>
#include <csignal>
#include <thread>
#include <atomic>
#include <chrono>

#ifdef _WIN32
#include <windows.h>
static void get_time(struct timespec *ts)
{
    static LARGE_INTEGER frequency;
    static BOOL use_qpc = QueryPerformanceFrequency(&frequency);
    LARGE_INTEGER count;
    QueryPerformanceCounter(&count);
    double seconds = (double)count.QuadPart / (double)frequency.QuadPart;
    ts->tv_sec = (time_t)seconds;
    ts->tv_nsec = (long)((seconds - ts->tv_sec) * 1e9);
}

static void nanosleep_win(const struct timespec *req)
{
    std::this_thread::sleep_for(std::chrono::seconds(req->tv_sec) + std::chrono::nanoseconds(req->tv_nsec));
}
#endif

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
#elif defined(_WIN32)
    nanosleep_win(&next);
#else
    // Fallback for other platforms
    struct timespec now;
#ifdef _WIN32
    get_time(&now);
#else
    clock_gettime(CLOCK_REALTIME, &now);
#endif
    long sec = next.tv_sec - now.tv_sec;
    long nsec = next.tv_nsec - now.tv_nsec;
    if (nsec < 0)
    {
        sec--;
        nsec += 1000000000;
    }
    if (sec > 0 || nsec > 0)
    {
#ifdef _WIN32
        struct timespec req = { sec, nsec };
        nanosleep_win(&req);
#else
        struct timespec req = { sec, nsec };
        nanosleep(&req, nullptr);
#endif
    }
#endif
}

void run_periodic(void (*func)())
{
    signal(SIGINT, sigint_handler);

    struct timespec next;
#ifdef _WIN32
    get_time(&next);
#else
    clock_gettime(CLOCK_MONOTONIC, &next);
#endif

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
