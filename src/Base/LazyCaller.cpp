/**
   @author Shin'ichiro Nakaoka
*/

#include "LazyCaller.h"
#include <QObject>
#include <QEvent>
#include <QCoreApplication>
#include <QThread>
#include <QSemaphore>
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
    CallEvent(const std::function<void(void)>& function)
        : QEvent(QEvent::User),
          function(function) {
    }
    CallEvent(const std::function<void(void)>& function, SyncInfoPtr& syncInfo)
        : QEvent(QEvent::User),
          function(function),
          syncInfo(syncInfo) {
    }
    CallEvent(const CallEvent& org)
        : QEvent(QEvent::User),
          function(org.function),
          syncInfo(org.syncInfo) {
    }
    ~CallEvent() {
        if(syncInfo){
            syncInfo->semaphore.release(); // wake up the caller process
        }
    }
    std::function<void(void)> function;
    SyncInfoPtr syncInfo;
};
    

class CallEventHandler : public QObject
{
public:
    CallEventHandler() {
        mainThreadId = QThread::currentThreadId();
    }
    virtual bool event(QEvent* e);
    Qt::HANDLE mainThreadId;
};
    
CallEventHandler callEventHandler;
}

namespace cnoid {

class LazyCallerImpl : public QObject
{
public:
    LazyCaller* self;
    std::function<void(void)> function;
    int priority;
    bool isConservative;
    LazyCallerImpl(LazyCaller* self);
    LazyCallerImpl(LazyCaller* self, const std::function<void(void)>& function, int priority);
    virtual bool event(QEvent* e);
};

class QueuedCallerImpl : public QObject
{
public:
    QueuedCaller* self;
    ~QueuedCallerImpl();
    virtual bool event(QEvent* e);
};

}


bool cnoid::isRunningInMainThread()
{
    return (QThread::currentThreadId() == callEventHandler.mainThreadId);
}


void cnoid::callLater(const std::function<void(void)>& function, int priority)
{
    CallEvent* event = new CallEvent(function);
    QCoreApplication::postEvent(&callEventHandler, event, toQtPriority(priority));
}


void cnoid::callFromMainThread(const std::function<void(void)>& function, int priority)
{
    if(QThread::currentThreadId() == callEventHandler.mainThreadId){
        function();
    } else {
        CallEvent* event = new CallEvent(function);
        QCoreApplication::postEvent(&callEventHandler, event, toQtPriority(priority));
    }
}


bool cnoid::callSynchronously(const std::function<void(void)>& function, int priority)
{
    if(QThread::currentThreadId() == callEventHandler.mainThreadId){
        function();
        //callLater(function, priority);
        return true;
    } else {
        SyncInfoPtr syncInfo = std::make_shared<SyncInfo>();
        QCoreApplication::postEvent(
            &callEventHandler, new CallEvent(function, syncInfo), toQtPriority(priority));
        syncInfo->semaphore.acquire(); // wait for finish
        return syncInfo->completed;
    }
}


bool CallEventHandler::event(QEvent* e)
{
    CallEvent* callEvent = dynamic_cast<CallEvent*>(e);
    if(callEvent){
        callEvent->function();
        if(callEvent->syncInfo){
            callEvent->syncInfo->completed = true;
        }
        return true;
    }
    return false;
}


LazyCaller::LazyCaller()
{
    isPending_ = false;
    impl = new LazyCallerImpl(this);
}


LazyCallerImpl::LazyCallerImpl(LazyCaller* self)
    : self(self)
{
    priority = LazyCaller::PRIORITY_HIGH;
    isConservative = false;
}
    

LazyCaller::LazyCaller(const std::function<void(void)>& function, int priority)
{
    isPending_ = false;
    impl = new LazyCallerImpl(this, function, priority);
}


LazyCallerImpl::LazyCallerImpl(LazyCaller* self, const std::function<void(void)>& function, int priority)
    : self(self),
      function(function),
      priority(priority)
{
    isConservative = false;
}


LazyCaller::LazyCaller(const LazyCaller& org)
{
    isPending_ = false;
    impl = new LazyCallerImpl(this, org.impl->function, org.impl->priority);
    impl->isConservative = org.impl->isConservative;
}


LazyCaller::~LazyCaller()
{
    cancel();
    delete impl;
}


void LazyCaller::setFunction(const std::function<void(void)>& function)
{
    impl->function = function;
}


void LazyCaller::setPriority(int priority)
{
    impl->priority = priority;
}


void LazyCaller::setConservative(bool on)
{
    impl->isConservative = on;
}


void LazyCaller::cancel()
{
    if(isPending_){
        QCoreApplication::removePostedEvents(impl);
        isPending_ = false;
    }
}


void LazyCaller::flush()
{
    if(isPending_){
        QCoreApplication::removePostedEvents(impl);
        isPending_ = false;
    }
    impl->function();
}


void LazyCaller::postCallEvent()
{
    CallEvent* event = new CallEvent(impl->function);
    QCoreApplication::postEvent(impl, event, toQtPriority(impl->priority));
}


bool LazyCallerImpl::event(QEvent* e)
{
    CallEvent* callEvent = dynamic_cast<CallEvent*>(e);
    if(callEvent){
        if(isConservative){
            callEvent->function();
            self->isPending_ = false;
        } else {
            self->isPending_ = false;
            callEvent->function();
        }
        return true;
    }
    return false;
}


QueuedCaller::QueuedCaller()
{
    impl = new QueuedCallerImpl();
}


QueuedCaller::~QueuedCaller()
{
    cancel();
    delete impl;
}


QueuedCallerImpl::~QueuedCallerImpl()
{
    
}


void QueuedCaller::callLater(const std::function<void()>& function, int priority)
{
    CallEvent* event = new CallEvent(function);
    QCoreApplication::postEvent(impl, event, toQtPriority(priority));
}


bool QueuedCallerImpl::event(QEvent* e)
{
    CallEvent* callEvent = dynamic_cast<CallEvent*>(e);
    if(callEvent){
        callEvent->function();
        return true;
    }
    return false;
}


void QueuedCaller::cancel()
{
    QCoreApplication::removePostedEvents(impl);
}
