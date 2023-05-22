#include "LazyCaller.h"
#include <QObject>
#include <QEvent>
#include <QCoreApplication>
#include <QThread>
#include <QSemaphore>
#include <mutex>
#include <memory>

using namespace std;
using namespace cnoid;

namespace {

inline int toQtPriority(int priority) {
    switch(priority){
    case LazyCaller::HighPriority:
        return Qt::HighEventPriority;
    case LazyCaller::NormalPriority:
        return Qt::NormalEventPriority;
    case LazyCaller::LowPriority:
        return Qt::LowEventPriority;
    case LazyCaller::MinimumPriority:
        return INT_MIN;
    default:
        break;
    }
    if(priority < LazyCaller::HighPriority){
        return Qt::HighEventPriority;
    } else {
        return INT_MIN;
    }
}

class SyncInfo
{
public:
    QSemaphore semaphore;
    bool completed;
    SyncInfo() {
        completed = false;
    }
};
typedef std::shared_ptr<SyncInfo> SyncInfoPtr;

class CallEvent : public QEvent
{
public:
    CallEvent()
        : QEvent(QEvent::User)
    { }
    CallEvent(const std::function<void()>& func)
        : QEvent(QEvent::User),
          func(func)
    { }
    CallEvent(const std::function<void(void)>& func, SyncInfoPtr& syncInfo)
        : QEvent(QEvent::User),
          func(func),
          syncInfo(syncInfo)
    { }
    CallEvent(const CallEvent& org)
        : QEvent(QEvent::User),
          func(org.func),
          syncInfo(org.syncInfo)
    { }
    ~CallEvent() {
        if(syncInfo){
            syncInfo->semaphore.release(); // wake up the caller process
        }
    }
    std::function<void(void)> func;
    SyncInfoPtr syncInfo;
};
    
class CallEventHandler : public QObject
{
public:
    CallEventHandler() {
        mainThreadId = QThread::currentThreadId();
    }
    virtual bool event(QEvent* e) override;
    Qt::HANDLE mainThreadId;
};
    
CallEventHandler callEventHandler;

}

namespace cnoid {

class LazyCaller::Impl : public QObject
{
public:
    LazyCaller* self;
    std::function<void()> func;
    int priority;
    
    Impl(LazyCaller* self);
    Impl(LazyCaller* self, const std::function<void(void)>& func, int priority);
    virtual bool event(QEvent* e) override;
};

class LazyOverwriteCaller::Impl : public QObject
{
public:
    LazyOverwriteCaller* self;
    std::function<void()> func;
    std::mutex funcMutex;
    bool isPending;
    int priority;
    
    Impl(LazyOverwriteCaller* self);
    virtual bool event(QEvent* e) override;
};

}


bool cnoid::isRunningInMainThread()
{
    return (QThread::currentThreadId() == callEventHandler.mainThreadId);
}


void cnoid::callLater(const std::function<void()>& func, int priority)
{
    auto event = new CallEvent(func);
    QCoreApplication::postEvent(&callEventHandler, event, toQtPriority(priority));
}


void cnoid::callFromMainThread(const std::function<void()>& func, int priority)
{
    if(QThread::currentThreadId() == callEventHandler.mainThreadId){
        func();
    } else {
        auto event = new CallEvent(func);
        QCoreApplication::postEvent(&callEventHandler, event, toQtPriority(priority));
    }
}


bool cnoid::callSynchronously(const std::function<void()>& func, int priority)
{
    if(QThread::currentThreadId() == callEventHandler.mainThreadId){
        func();
        return true;
    } else {
        auto syncInfo = std::make_shared<SyncInfo>();
        QCoreApplication::postEvent(
            &callEventHandler, new CallEvent(func, syncInfo), toQtPriority(priority));
        syncInfo->semaphore.acquire(); // wait for finish
        return syncInfo->completed;
    }
}


bool CallEventHandler::event(QEvent* e)
{
    if(auto callEvent = dynamic_cast<CallEvent*>(e)){
        callEvent->func();
        if(callEvent->syncInfo){
            callEvent->syncInfo->completed = true;
        }
        return true;
    }
    return false;
}


LazyCaller::LazyCaller()
    : isPending_(false)
{
    impl = new Impl(this);
}


LazyCaller::Impl::Impl(LazyCaller* self)
    : Impl(self, nullptr, HighPriority)
{

}
    

LazyCaller::LazyCaller(const std::function<void()>& func, int priority)
    : isPending_(false)    
{
    impl = new Impl(this, func, priority);
}


LazyCaller::Impl::Impl(LazyCaller* self, const std::function<void()>& func, int priority)
    : self(self),
      func(func),
      priority(priority)
{

}


LazyCaller::LazyCaller(const LazyCaller& org)
    : isPending_(false)    
{
    impl = new Impl(this, org.impl->func, org.impl->priority);
}


LazyCaller::~LazyCaller()
{
    cancel();
    delete impl;
}


void LazyCaller::setFunction(const std::function<void()>& func)
{
    impl->func = func;
}


bool LazyCaller::hasFunction() const
{
    return impl->func != nullptr;
}


void LazyCaller::setPriority(int priority)
{
    impl->priority = priority;
}


void LazyCaller::postCallEvent()
{
    QCoreApplication::postEvent(impl, new CallEvent(impl->func), toQtPriority(impl->priority));
}


bool LazyCaller::Impl::event(QEvent* e)
{
    if(auto callEvent = dynamic_cast<CallEvent*>(e)){
        self->isPending_.store(false);
        callEvent->func();
        return true;
    }
    return false;
}


void LazyCaller::cancel()
{
    QCoreApplication::removePostedEvents(impl);
    isPending_.store(false);
}


void LazyCaller::flush()
{
    cancel();

    if(impl->func){
        impl->func();
    }
}


LazyOverwriteCaller::LazyOverwriteCaller()
{
    impl = new Impl(this);
}


LazyOverwriteCaller::Impl::Impl(LazyOverwriteCaller* self)
    : self(self)
{
    isPending = false;
    priority = HighPriority;
}


LazyOverwriteCaller::~LazyOverwriteCaller()
{
    cancel();
    delete impl;
}


void LazyOverwriteCaller::setPriority(int priority)
{
    impl->priority = priority;
}


void LazyOverwriteCaller::callLater(const std::function<void()>& func)
{
    bool doPost = false;
    {
        lock_guard<mutex> guard(impl->funcMutex);
        impl->func = func;
        if(!impl->isPending){
            doPost = true;
            impl->isPending = true;
        }
    }
    if(doPost){
        QCoreApplication::postEvent(impl, new CallEvent, toQtPriority(impl->priority));
    }
}


bool LazyOverwriteCaller::Impl::event(QEvent* e)
{
    if(auto callEvent = dynamic_cast<CallEvent*>(e)){
        std::function<void()> duplicatedFunc;
        {
            isPending = false;
            lock_guard<mutex> guard(funcMutex);
            duplicatedFunc = func;
        }
        duplicatedFunc();
        return true;
    }
    return false;
}


void LazyOverwriteCaller::cancel()
{
    QCoreApplication::removePostedEvents(impl);

    {
        lock_guard<mutex> guard(impl->funcMutex);
        impl->isPending = false;
    }
}


void LazyOverwriteCaller::flush()
{
    cancel();

    if(impl->func){
        impl->func();
    }
}
