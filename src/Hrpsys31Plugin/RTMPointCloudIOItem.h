/**
   \file
   \author Shizuko Hattori
*/

#ifndef CNOID_HRPSYS31_PLUGIN_RTMPOINTCLOUDIO_ITEM_H_INCLUDED
#define CNOID_HRPSYS31_PLUGIN_RTMPOINTCLOUDIO_ITEM_H_INCLUDED

#include <cnoid/Item>
#include <cnoid/BodyItem>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class RTMPointCloudIOItemImpl;

class CNOID_EXPORT RTMPointCloudIOItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    RTMPointCloudIOItem();
    RTMPointCloudIOItem(const RTMPointCloudIOItem& org);
    ~RTMPointCloudIOItem();

    virtual void onPositionChanged();
    virtual void onDisconnectedFromRoot();
    virtual Item* doDuplicate() const;
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    RTMPointCloudIOItemImpl* impl;
    BodyItem* bodyItem;
};

typedef ref_ptr<RTMPointCloudIOItem> RTMPointCloudIOItemPtr;

}

#endif /* CNOID_HRPSYS31_PLUGIN_RTMPOINTCLOUDIO_ITEM_H_INCLUDED */
