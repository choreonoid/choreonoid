#ifndef CNOID_BASE_LOCATABLE_ITEM_H
#define CNOID_BASE_LOCATABLE_ITEM_H

#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LocatableItem
{
public:
    virtual SignalProxy<void()> sigLocationChanged() = 0;
    virtual Position getLocation() const = 0;
    virtual void setLocation(const Position& T) = 0;
    virtual bool isLocationEditable() const = 0;
    virtual bool hasParentLocation() const;
    virtual std::string getParentLocationName() const;
    virtual Position getParentLocation() const;
};

}

#endif
