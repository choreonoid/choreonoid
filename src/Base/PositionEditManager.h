#ifndef CNOID_BASE_POSITION_EDIT_MANAGER_H
#define CNOID_BASE_POSITION_EDIT_MANAGER_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class AbstractPositionEditTarget : public Referenced
{
public:
    virtual std::string name() = 0;
    virtual Position getPosition() = 0;
    virtual bool setPosition(const Position& T) = 0;
    virtual SignalProxy<void(const Position& T)> sigPositionChanged() = 0;
    virtual SignalProxy<void()> sigFinishRequest() = 0;
};

typedef ref_ptr<AbstractPositionEditTarget> AbstractPositionEditTargetPtr;
    
class CNOID_EXPORT PositionEditManager
{
public:
    static PositionEditManager* instance();

    SignalProxy<bool(AbstractPositionEditTarget* target), LogicalSum> sigPositionEditRequest();
    SignalProxy<void(AbstractPositionEditTarget* target)> sigFinishRequest();

    AbstractPositionEditTarget* lastPositionEditTarget();

    bool requestPositionEdit(AbstractPositionEditTarget* target);

private:
    PositionEditManager();
    ~PositionEditManager();

    class Impl;
    Impl* impl;
};

}

#endif
