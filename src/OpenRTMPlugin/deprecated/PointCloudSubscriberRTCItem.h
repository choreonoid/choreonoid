/**
   \file
   \author Shizuko Hattori
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_OPENRTM_PLUGIN_POINTCLOUD_SUBSCRIBER_RTC_ITEM_H
#define CNOID_OPENRTM_PLUGIN_POINTCLOUD_SUBSCRIBER_RTC_ITEM_H

#include <cnoid/Item>
#include <cnoid/BodyItem>
#include <cnoid/ControllerItem>

namespace cnoid {

class PointCloudSubscriberRTCItemImpl;

class PointCloudSubscriberRTCItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    PointCloudSubscriberRTCItem();
    PointCloudSubscriberRTCItem(const PointCloudSubscriberRTCItem& org);
    ~PointCloudSubscriberRTCItem();

    virtual void onPositionChanged();
    virtual void onDisconnectedFromRoot();
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
    PointCloudSubscriberRTCItemImpl* impl;
    BodyItem* bodyItem;
};

typedef ref_ptr<PointCloudSubscriberRTCItem> PointCloudSubscriberRTCItemPtr;

}

#endif
