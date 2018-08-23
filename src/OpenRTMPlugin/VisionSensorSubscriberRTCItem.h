/**
   \file
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_VISION_SENSOR_SUBSCRIBER_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_VISION_SENSOR_SUBSCRIBER_RTC_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class VisionSensorSubscriberRTCItemImpl;

class CNOID_EXPORT VisionSensorSubscriberRTCItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    VisionSensorSubscriberRTCItem();
    VisionSensorSubscriberRTCItem(const VisionSensorSubscriberRTCItem& org);
    ~VisionSensorSubscriberRTCItem();

    void setPeriodicRate(int rate);

    virtual void onDisconnectedFromRoot();
    virtual void onPositionChanged();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    VisionSensorSubscriberRTCItemImpl* impl;
};

typedef ref_ptr<VisionSensorSubscriberRTCItem> VisionSensorSubscriberRTCItemPtr;

}

#endif
