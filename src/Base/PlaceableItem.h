#ifndef CNOID_BASE_PLACEABLE_ITEM_H
#define CNOID_BASE_PLACEABLE_ITEM_H

#include <cnoid/Signal>

namespace cnoid {

class PlaceableItem
{
public:
    virtual SignalProxy<void()> sigLocationChanged() = 0;
    virtual Position getLocation() const = 0;
    virtual void setLocation(const Position& T) = 0;
    virtual bool isLocationEditable() const = 0;
};

}

#endif
