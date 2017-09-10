#pragma once
#include <signal.h>
#include <time.h>
#define CLOCKID CLOCK_REALTIME
#define SIG SIGRTMIN
void begin(double sample_time);
volatile int execute=1;
double real_period=0;
static void handler(int sig, siginfo_t *si, void *uc);
long long freq_nanosecs;
timer_t timerid;
struct sigevent sev;
struct itimerspec its;
struct timespec real_time_clock;
struct timespec real_time_clock_begin;
struct timespec real_time_clock_end;
sigset_t mask;
struct sigaction sa;
double rt_clock_begin=0;
double rt_clock_end=0;
volatile double rt_clock=0;
volatile double timer=0;
    static void handler(int sig, siginfo_t *si, void *uc){
        clock_gettime(CLOCK_REALTIME,&real_time_clock);
        rt_clock=real_time_clock.tv_sec+(double)real_time_clock.tv_nsec/1000000000.0f;
        real_period=rt_clock-timer;
        timer=rt_clock;
        execute=1;
    }
    void timer_begin(double sample_time){
        sa.sa_flags = SA_SIGINFO;
        sa.sa_sigaction = handler;
        sigemptyset(&sa.sa_mask);
        sigaction(SIG, &sa, NULL);
        sigemptyset(&mask);
        sigaddset(&mask, SIG);
        sigprocmask(SIG_SETMASK, &mask, NULL);
        sev.sigev_notify = SIGEV_SIGNAL;
        sev.sigev_signo = SIG;
        sev.sigev_value.sival_ptr = &timerid;
        timer_create(CLOCKID, &sev, &timerid);
        freq_nanosecs=(long long)(sample_time*1000000000);
        its.it_value.tv_sec = freq_nanosecs / 1000000000;
        its.it_value.tv_nsec = freq_nanosecs % 1000000000;
        its.it_interval.tv_sec = its.it_value.tv_sec;
        its.it_interval.tv_nsec = its.it_value.tv_nsec;
        timer_settime(timerid, 0, &its, NULL);
        sigprocmask(SIG_UNBLOCK, &mask, NULL);
    }
