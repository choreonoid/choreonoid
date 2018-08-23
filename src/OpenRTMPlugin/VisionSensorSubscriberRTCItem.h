/**
   \file
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_VISION_SENSOR_SUBSCRIBER_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_VISION_SENSOR_SUBSCRIBER_RTC_ITEM_H

#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class VisionSensorSubscriberRTCItemImpl;

class CNOID_EXPORT VisionSensorSubscriberRTCItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    VisionSensorSubscriberRTCItem();
    VisionSensorSubscriberRTCItem(const VisionSensorSubscriberRTCItem& org);
    ~VisionSensorSubscriberRTCItem();

    virtual void onDisconnectedFromRoot();
    virtual void onPositionChanged();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

    virtual bool start() override;
    virtual double timeStep() const override;
    virtual void input() override;
    virtual bool control() override;
    virtual void output() override;
    virtual void stop() override;

private:
    VisionSensorSubscriberRTCItemImpl* impl;
};

typedef ref_ptr<VisionSensorSubscriberRTCItem> VisionSensorSubscriberRTCItemPtr;

}

#endif
