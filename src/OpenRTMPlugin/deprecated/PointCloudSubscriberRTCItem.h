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

    virtual void onPositionChanged() override;
    virtual void onDisconnectedFromRoot() override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

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
