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
    enum LocationType {
        InvalidLocation,
        GlobalLocation,
        ParentRelativeLocation,
        // This is basically same as ParentRelativeCoordinate, but
        // this maeks the global coordinate unavailable in the user interface
        OffsetLocation
    };

    virtual ~LocationProxy();

    LocationType locationType() const { return locationType_; }
    void setLocationType(LocationType type) { locationType_ = type; }
    virtual std::string getName() const;
    virtual Isometry3 getLocation() const = 0;
    virtual bool isEditable() const;
    virtual void setEditable(bool on);
    virtual bool setLocation(const Isometry3& T);
    virtual void finishLocationEditing();
    virtual Item* getCorrespondingItem();
    virtual LocationProxyPtr getParentLocationProxy() const;
    virtual void expire();
    virtual SignalProxy<void()> sigLocationChanged() = 0;
    virtual SignalProxy<void()> sigAttributeChanged();
    virtual SignalProxy<void()> sigExpired();
    Isometry3 getGlobalLocation() const;
    Isometry3 getGlobalLocationOf(const Isometry3 T) const;
    bool setGlobalLocation(const Isometry3& T);
    void notifyAttributeChange();
    bool requestEdit();

    static SignalProxy<bool(LocationProxyPtr location), LogicalSum> sigEditRequest();

protected:
    LocationProxy(LocationType type);

private:
    LocationType locationType_;
    bool isEditable_;
    Signal<void()> sigAttributeChanged_;
    Signal<void()> sigExpired_;
    ScopedConnection itemNameConnection_;
};

class LocatableItem
{
public:
    virtual LocationProxyPtr getLocationProxy() = 0;
};

}

#endif
