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
    virtual ~LocationProxy();

    Item* locatableItem() { return locatableItem_; }

    enum LocationType {
        InvalidLocation,
        GlobalLocation,
        ParentRelativeLocation,
        // This is basically same as ParentRelativeCoordinate, but
        // this maeks the global coordinate unavailable in the user interface
        OffsetLocation
    };
    LocationType locationType() const { return locationType_; }
    void setLocationType(LocationType type) { locationType_ = type; }
    
    virtual std::string getName() const;

    /**
       This function usually returns an empty string.
       If the category is specified, only the proxies that have the same category
       can be handled simultaneously in the location view.
    */
    virtual std::string getCategory() const;
    
    virtual Isometry3 getLocation() const = 0;
    virtual bool isLocked() const;
    virtual void setLocked(bool on);
    virtual bool isContinuousUpdateState() const;
    virtual bool setLocation(const Isometry3& T);
    virtual void finishLocationEditing();
    virtual LocationProxyPtr getParentLocationProxy();
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

    [[deprecated("Use isContinuousUpdateState.")]]
    bool isDoingContinuousUpdate() const { return isContinuousUpdateState(); }
    
    [[deprecated("Use locatableItem.")]]
    Item* getCorrespondingItem() { return locatableItem(); }

protected:
    LocationProxy(Item* locatableItem, LocationType type);
    void setNameDependencyOnItemName();

private:
    Item* locatableItem_;
    LocationType locationType_;
    bool isLocked_;
    Signal<void()> sigAttributeChanged_;
    Signal<void()> sigExpired_;
    ScopedConnection itemConnection;
    ScopedConnection itemNameConnection;
    ScopedConnection continuousUpdateStateConnection;
};

class CNOID_EXPORT LocatableItem
{
public:
    virtual LocationProxyPtr getLocationProxy();

    /**
       \note The proxies returned by this function are used in preference to
       the one obtained by the getLocationProxy function.
    */
    virtual std::vector<LocationProxyPtr> getLocationProxies();

    /**
       This signal notifies the slots of the change of the location proxies.
       Override this function to return a valid signal proxy if the location proxies
       may be changed while the item is being selected.
    */
    virtual SignalProxy<void()> getSigLocationProxiesChanged();
};

}

#endif
