/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_OPENRTM_PLUGIN_OLD_RTM_POINTCLOUD_IO_ITEM_H
#define CNOID_OPENRTM_PLUGIN_OLD_RTM_POINTCLOUD_IO_ITEM_H

#include <cnoid/Item>
#include <cnoid/BodyItem>
#include <cnoid/SceneProvider>
#include <cnoid/ControllerItem>
#include "exportdecl.h"

namespace cnoid {

class OldRTMPointCloudIOItemImpl;

class CNOID_EXPORT OldRTMPointCloudIOItem : public ControllerItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    OldRTMPointCloudIOItem();
    OldRTMPointCloudIOItem(const OldRTMPointCloudIOItem& org);
    ~OldRTMPointCloudIOItem();

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
    OldRTMPointCloudIOItemImpl* impl;
    BodyItem* bodyItem;
};

typedef ref_ptr<OldRTMPointCloudIOItem> OldRTMPointCloudIOItemPtr;

}

#endif
