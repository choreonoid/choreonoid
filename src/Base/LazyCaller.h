/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LAZY_CALLER_H
#define CNOID_BASE_LAZY_CALLER_H

#include <functional>
#include "exportdecl.h"

namespace cnoid {
    
class LazyCallerImpl;


/**
   \note This is not thread safe
   \todo Make this thread safe so that the function can be called non-main threads
*/
class CNOID_EXPORT LazyCaller
{
    friend class LazyCallerImpl;
        
    bool isPending_;
    bool isNextPending;
    LazyCallerImpl* impl;
        
public:
    enum Priority {
        HighPriority = 0,
        NormalPriority,
        LowPriority,
        MinimumPriority,

        // deprecated
        PRIORITY_HIGH = HighPriority,
        PRIORITY_NORMAL = NormalPriority,
        PRIORITY_LOW = LowPriority
    };
        
    LazyCaller();
    LazyCaller(const std::function<void(void)>& function, int priority = HighPriority);
    LazyCaller(const LazyCaller& org);
    virtual ~LazyCaller();

    void setFunction(const std::function<void(void)>& function);
    bool hasFunction() const;
    void setPriority(int priority);

    /**
       The function is called once even if the lazy call is requested many times
       before the function is actually called.
       If the conservative mode is on, a function call request is ignored if the function
       previously requested to call is being executing.
    */
    void setConservative(bool on);

    bool isPending() const { return isPending_; }

    void flush();

    typedef void result_type;

    /**
       Multiple requests before the actual function call is summarized into a single call
    */
    void operator()() {
        if(!isPending_){
            isPending_ = true;
            postCallEvent();
        }
    }

    void cancel();

private:
    void postCallEvent();
};


class QueuedCallerImpl;

class CNOID_EXPORT QueuedCaller
{
    QueuedCallerImpl* impl;

    QueuedCaller(const QueuedCaller& org);
    
  public:
    /**
       All the queued functions that have not been executed are canceled in the destructor.
    */
    QueuedCaller();
    virtual ~QueuedCaller();
    
    void callLater(const std::function<void()>& function, int priority = LazyCaller::NormalPriority);

    void cancel();
};

    
//! deprecated
enum { IDLE_PRIORITY_HIGH = LazyCaller::PRIORITY_HIGH,
       IDLE_PRIORITY_NORMAL = LazyCaller::PRIORITY_NORMAL,
       IDLE_PRIORITY_LOW = LazyCaller::PRIORITY_LOW };

CNOID_EXPORT void callLater(const std::function<void()>& function, int priority = LazyCaller::NormalPriority);
CNOID_EXPORT void callFromMainThread(const std::function<void()>& function, int priority = LazyCaller::NormalPriority);
CNOID_EXPORT bool callSynchronously(const std::function<void()>& function, int priority = LazyCaller::NormalPriority);

CNOID_EXPORT bool isRunningInMainThread();

}
        
#endif
