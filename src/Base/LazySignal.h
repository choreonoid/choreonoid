/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_LAZY_SIGNAL_H_INCLUDED
#define CNOID_BASE_LAZY_SIGNAL_H_INCLUDED

#include "LazyCaller.h"
#include <boost/signal.hpp>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LazySignalBase : public LazyCaller
{
public:
    void request();
    void requestBlocking(boost::signals::connection connection){
        connectionsToBlock.push_back(connection);
    }
protected:
    LazySignalBase();
    LazySignalBase(boost::function<void()> emitFunction, int priority);
    boost::function<void()> emitFunction;
    std::vector<boost::signals::connection> connectionsToBlock;
    virtual void defaultEmitFunction() = 0;

private:
    bool doEmit();
};

template <class SignalType> class LazySignal : public LazySignalBase
{
public:
    LazySignal() { }

    LazySignal(boost::function<void()> emitFunction, int priority = LazyCaller::PRIORITY_HIGH)
        : LazySignalBase(emitFunction, priority) {
    }

    SignalType& signal() { return signal_; }

protected:
    virtual void defaultEmitFunction() {
        signal_();
    }

private:
    SignalType signal_;
};
}
        
#endif
