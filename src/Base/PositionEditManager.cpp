#include "PositionEditManager.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class PositionEditManager::Impl
{
public:
    Signal<bool(AbstractPositionEditTarget* target), LogicalSum> sigPositionEditRequest;
    Signal<void(AbstractPositionEditTarget* target)> sigFinishRequest;
    AbstractPositionEditTargetPtr lastTarget;
    ScopedConnection finishRequestConnection;

    void onFinishRequest(AbstractPositionEditTarget* target);
    bool requestPositionEdit(AbstractPositionEditTarget* target);
};

}


PositionEditManager* PositionEditManager::instance()
{
    static PositionEditManager manager;
    return &manager;
}


PositionEditManager::PositionEditManager()
{
    impl = new Impl;
}


PositionEditManager::~PositionEditManager()
{
    delete impl;
}


SignalProxy<bool(AbstractPositionEditTarget* target), LogicalSum> PositionEditManager::sigPositionEditRequest()
{
    return impl->sigPositionEditRequest;
}


AbstractPositionEditTarget* PositionEditManager::lastPositionEditTarget()
{
    return impl->lastTarget;
}


bool PositionEditManager::requestPositionEdit(AbstractPositionEditTarget* target)
{
    return impl->requestPositionEdit(target);
}


bool PositionEditManager::Impl::requestPositionEdit(AbstractPositionEditTarget* target)
{
    if(lastTarget && target != lastTarget){
        sigFinishRequest(lastTarget);
    }
        
    bool accepted = sigPositionEditRequest(target);
    
    lastTarget = target;

    finishRequestConnection =
        target->sigFinishRequest().connect([this, target](){ onFinishRequest(target); });
    
    return accepted;
}


void PositionEditManager::Impl::onFinishRequest(AbstractPositionEditTarget* target)
{
    finishRequestConnection.disconnect();

    sigFinishRequest(target);
    
    if(target == lastTarget){
        lastTarget = nullptr;
    }
}
