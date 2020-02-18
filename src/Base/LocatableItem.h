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
    virtual Item* getCorrespondingItem();
    virtual std::string getLocationName() const;
    virtual Position getLocation() const = 0;
    virtual bool prefersLocalLocation() const;
    virtual bool getLocationEditable() const;
    virtual void setLocationEditable(bool on);
    virtual SignalProxy<void(bool on)> sigLocationEditableToggled();
    virtual void setLocation(const Position& T) = 0;
    virtual SignalProxy<void()> sigLocationChanged() = 0;
    virtual LocatableItem* getParentLocatableItem();
    
private:
    bool isLocationEditable_;
    Signal<void(bool on)> sigLocationEditableToggled_;
};

}

#endif
