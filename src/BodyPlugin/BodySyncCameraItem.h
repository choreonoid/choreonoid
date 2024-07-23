#ifndef CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_ITEM_H

#include <cnoid/CameraItem>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class Link;

class CNOID_EXPORT BodySyncCameraItem : public CameraItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    static BodySyncCameraItem* showDialogToCreateBodySyncCameraItem(BodyItem* bodyItem, Link* link);

    BodySyncCameraItem();
    virtual ~BodySyncCameraItem();

    void setTargetLink(const std::string& name);
    const std::string& targetLinkName() const;
    Isometry3 targetLinkPosition() const;
    void setParallelTrackingMode(bool on);
    bool isParallelTrackingMode() const;
    void updateRelativeCameraPosition();
    Isometry3 relativeCameraPosition() const;
    
    virtual void showDialogToConfigureCamera() override;

protected:
    BodySyncCameraItem(const BodySyncCameraItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodySyncCameraItem> BodySyncCameraItemPtr;

}

#endif
