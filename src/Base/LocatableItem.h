#ifndef CNOID_BASE_LOCATABLE_ITEM_H
#define CNOID_BASE_LOCATABLE_ITEM_H

#include <cnoid/Referenced>
#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class Item;
class LocationProxy;
typedef ref_ptr<LocationProxy> LocationProxyPtr;

class CNOID_EXPORT LocationProxy : public Referenced
{
public:
    LocationProxy();
    virtual ~LocationProxy();

    enum LocationType {
        InvalidLocation,
        GlobalLocation,
        ParentRelativeLocation,
        OffsetLocation
    };
    virtual int getType() const = 0;
    virtual std::string getName() const;
    virtual Position getLocation() const = 0;
    virtual bool isEditable() const;
    virtual void setEditable(bool on);
    virtual void setLocation(const Position& T);
    virtual Item* getCorrespondingItem();
    virtual LocationProxyPtr getParentLocationProxy();
    virtual void expire();
    virtual SignalProxy<void()> sigLocationChanged() = 0;
    virtual SignalProxy<void()> sigAttributeChanged();
    virtual SignalProxy<void()> sigExpired();
    void notifyAttributeChange();
    bool requestEdit();

    static SignalProxy<bool(LocationProxyPtr location), LogicalSum> sigEditRequest();

private:
    bool isEditable_;
    Signal<void()> sigAttributeChanged_;
    Signal<void()> sigExpired_;
};

class LocatableItem
{
public:
    virtual LocationProxyPtr getLocationProxy() = 0;
};

}

#endif
