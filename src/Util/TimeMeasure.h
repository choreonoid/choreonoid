
#ifndef CNOID_UTIL_TIME_MEASURE_H
#define CNOID_UTIL_TIME_MEASURE_H

#ifndef _WIN32

#include <time.h>
#ifdef _POSIX_C_SOURCE
#if _POSIX_C_SOURCE >= 199309L
#define USE_GETTIME
#endif
#endif

namespace cnoid {

class TimeMeasure
{
#ifdef USE_GETTIME
    struct timespec tp;
#else
    struct timeval tv;
#endif
    double time_;
    double totalTime_;
    int numCalls;
    
public:
    inline TimeMeasure() {
        totalTime_ = 0.0;
        numCalls = 0;
    }

    inline void begin() {
#ifdef USE_GETTIME
        clock_gettime(CLOCK_MONOTONIC, &tp) ;
#else
        gettimeofday(&tv, 0);
#endif
    }

    inline void end(){
#ifdef USE_GETTIME
        double beginTime = tp.tv_sec + (double)tp.tv_nsec * 1.0e-9;
        clock_gettime(CLOCK_MONOTONIC, &tp);
        double endTime = tp.tv_sec + (double)tp.tv_nsec * 1.0e-9;
#else
        double beginTime = tv.tv_sec + (double)tv.tv_usec * 1.0e-6;
        gettimeofday(&tv, 0);
        double endTime = tv.tv_sec + (double)tv.tv_usec * 1.0e-6;
#endif
        time_ = endTime - beginTime;
        totalTime_ += time_;
        numCalls++;
    }

    inline double measure() {
        end();
        return time_;
    }

    inline double time() { return time_; }
    inline double totalTime() { return totalTime_; }
    inline double avarageTime() { return totalTime_ / numCalls; }

};
}

#else
#include <windows.h>

namespace cnoid {
    
typedef unsigned __int64    ulonglong;

class TimeMeasure
{
    ulonglong iTimerScale;
    ulonglong beginTime;
    ulonglong endTime;
    double time_;
    double totalTime_;
    int numCalls;
 
public:
    inline TimeMeasure() { 
        totalTime_ = 0.0;
        numCalls = 0;
        BOOL iDummyBool = QueryPerformanceFrequency ((LARGE_INTEGER *) &iTimerScale);
        if(!iDummyBool)
            iTimerScale=1;
    }

    inline void begin() { 
        BOOL iDummyBool = QueryPerformanceCounter ((LARGE_INTEGER *) &beginTime);
        if(!iDummyBool)
            beginTime=1;
    }

    inline void end(){ 
        BOOL iDummyBool = QueryPerformanceCounter ((LARGE_INTEGER *) &endTime);
        if(!iDummyBool)
            endTime=0;
        time_ = (double)(endTime - beginTime) / iTimerScale;
        totalTime_ += time_;
        numCalls++;
    }
    inline double measure() {
        end();
        return time_;
    }
    inline double time() { return time_; }
    inline double totalTime() { return totalTime_; }
    inline double avarageTime() { return totalTime_ / numCalls; }
};

}

#endif

#endif
