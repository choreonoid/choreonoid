#ifndef CNOID_BODY_BODY_COLLISION_LINK_FILTER_H
#define CNOID_BODY_BODY_COLLISION_LINK_FILTER_H

#include <functional>
#include "exportdecl.h"

namespace cnoid {

class Body;
class Link;

class CNOID_EXPORT BodyCollisionLinkFilter
{
public:
    BodyCollisionLinkFilter();
    virtual ~BodyCollisionLinkFilter();

    typedef std::function<void(int linkIndex1, int linkIndex2)> FuncToDisableLinkPair;
    void setFuncToDisableLinkPair(FuncToDisableLinkPair func);
    void setTargetBody(Body* body, bool isSelfCollisionDetectionEnabled);
    bool checkIfEnabledLinkIndex(int linkIndex) const;
    void apply();
    void apply(FuncToDisableLinkPair func);

private:
    class Impl;
    Impl* impl;
    
};

}

#endif
