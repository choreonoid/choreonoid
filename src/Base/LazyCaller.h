#ifndef CNOID_BASE_LAZY_CALLER_H
#define CNOID_BASE_LAZY_CALLER_H

#include <functional>
#include <atomic>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LazyCallerBase
{
public:
    enum Priority {
        HighPriority = 0,
        NormalPriority,
        LowPriority,
        MinimumPriority,

        PRIORITY_HIGH [[deprecated]] = HighPriority,
        PRIORITY_NORMAL [[deprecated]] = NormalPriority,
        PRIORITY_LOW [[deprecated]] = LowPriority
    };
};

class CNOID_EXPORT LazyCaller : public LazyCallerBase
{
public:
    LazyCaller();
    LazyCaller(const std::function<void()>& function, int priority = HighPriority);
    LazyCaller(const LazyCaller& org);
    ~LazyCaller();

    void setFunction(const std::function<void()>& function);
    bool hasFunction() const;
    void setPriority(int priority);

    bool isPending() const {
        return isPending_.load();
    }

   /**
       Call the assigned function from the main thread later with the assigned priority.
       Duplicate requests before the actual function call is ignored.
    */
    void operator()(){
        bool expected = false;
        if(isPending_.compare_exchange_strong(expected, true)){
            postCallEvent();
        }
    }
    
    void cancel();
    void flush();

private:
    void postCallEvent();
    
    std::atomic<bool> isPending_;
    class Impl;
    Impl* impl;
};


class CNOID_EXPORT LazyOverwriteCaller : public LazyCallerBase
{
public:
    LazyOverwriteCaller();
    LazyOverwriteCaller(const LazyOverwriteCaller& org) = delete;
    ~LazyOverwriteCaller();
    
    void setPriority(int priority);

    /**
       Call an arbitrary function specified as the argument from the main thread later.
       For multiple requests before the actual function call, only the function of the
       last request is called at the timing of the first request.
    */
    void callLater(const std::function<void()>& function);

    void cancel();
    void flush();
    
private:
    class Impl;
    Impl* impl;
};

CNOID_EXPORT void callLater(const std::function<void()>& function, int priority = LazyCaller::NormalPriority);
CNOID_EXPORT void callFromMainThread(const std::function<void()>& function, int priority = LazyCaller::NormalPriority);
CNOID_EXPORT bool callSynchronously(const std::function<void()>& function, int priority = LazyCaller::NormalPriority);
CNOID_EXPORT bool isRunningInMainThread();

}
        
#endif
