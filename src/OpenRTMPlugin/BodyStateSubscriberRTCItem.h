/**
   \file
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_BODY_STATE_SUBSCRIBER_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_BODY_STATE_SUBSCRIBER_RTC_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class BodyStateSubscriberRTCItemImpl;

class CNOID_EXPORT BodyStateSubscriberRTCItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyStateSubscriberRTCItem();
    BodyStateSubscriberRTCItem(const BodyStateSubscriberRTCItem& org);
    ~BodyStateSubscriberRTCItem();

    void setPeriodicRate(int rate);

    enum PointCloudPortType {
        POINT_CLOUD_TYPES_POINT_CLOUD_TYPE,
        RTC_POINT_CLOUD_TYPE,
        N_POINT_CLOUD_PORT_TYPES
    };
    
    void setPointCloudPortType(int type);

    virtual void onDisconnectedFromRoot();
    virtual void onPositionChanged();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    BodyStateSubscriberRTCItemImpl* impl;
};

typedef ref_ptr<BodyStateSubscriberRTCItem> BodyStateSubscriberRTCItemPtr;

}

#endif
