#ifndef CNOID_BASE_LOCATABLE_ITEM_H
#define CNOID_BASE_LOCATABLE_ITEM_H

#include <cnoid/Signal>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LocatableItem
{
public:
    LocatableItem();
    virtual ~LocatableItem();

    enum LocationType {
        InvalidLocation,
        GlobalLocation,
        ParentRelativeLocation,
        OffsetLocation
    };
    virtual int getLocationType() const = 0;
    virtual LocatableItem* getParentLocatableItem();
    virtual Item* getCorrespondingItem();
    virtual std::string getLocationName() const;
    virtual Position getLocation() const = 0;
    virtual bool isLocationEditable() const;
    virtual void setLocationEditable(bool on);
    virtual void setLocation(const Position& T);
    virtual void expireLocation();
    virtual SignalProxy<void()> sigLocationChanged() = 0;
    virtual SignalProxy<void()> sigLocationAttributeChanged();
    virtual SignalProxy<void()> sigLocationExpired();

    bool requestLocationEdit();
    static SignalProxy<bool(LocatableItem* item), LogicalSum> sigLocationEditRequest();
    
private:
    bool isLocationEditable_;
    Signal<void()> sigLocationAttributeChanged_;
    Signal<void()> sigLocationExpired_;
};

}

#endif
