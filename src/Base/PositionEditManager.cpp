#include "PositionEditManager.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class PositionEditManager::Impl
{
public:
    Signal<bool(AbstractPositionEditTarget* target), LogicalSum> sigPositionEditRequest;
    AbstractPositionEditTarget* lastTarget;
    ScopedConnection expireConnection;

    void onPositionEditExpired(AbstractPositionEditTarget* target);
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
    impl->lastTarget = nullptr;
}


PositionEditManager::~PositionEditManager()
{
    delete impl;
}


SignalProxy<bool(AbstractPositionEditTarget* target), LogicalSum> PositionEditManager::sigPositionEditRequest()
{
    return impl->sigPositionEditRequest;
}


bool PositionEditManager::requestPositionEdit(AbstractPositionEditTarget* target)
{
    return impl->requestPositionEdit(target);
}


bool PositionEditManager::Impl::requestPositionEdit(AbstractPositionEditTarget* target)
{
    bool accepted = sigPositionEditRequest(target);
    
    lastTarget = target;

    expireConnection =
        target->sigPositionEditTargetExpired().connect(
            [this, target](){ onPositionEditExpired(target); });
    
    return accepted;
}


void PositionEditManager::Impl::onPositionEditExpired(AbstractPositionEditTarget* target)
{
    expireConnection.disconnect();

    if(target == lastTarget){
        lastTarget = nullptr;
    }
}


AbstractPositionEditTarget* PositionEditManager::lastPositionEditTarget()
{
    return impl->lastTarget;
}
