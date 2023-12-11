#ifndef CNOID_BODY_PLUGIN_BODY_POSE_LIST_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_POSE_LIST_ITEM_H

#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class BodyPoseItem;
class Listing;

class CNOID_EXPORT BodyPoseListItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);

    BodyPoseListItem();
    BodyPoseListItem(const BodyPoseListItem& org);
    ~BodyPoseListItem();
    virtual std::string displayName() const override;

    BodyItem* targetBodyItem() const { return targetBodyItem_; }

    enum PoseListArchiveMode {
        PoseItemArchiveMode,
        PoseListFileArchiveMode
    };

    int poseListArchiveMode() const { return poseListArchiveMode_; }
    void setPoseListArchiveMode(int mode);
    
    void addPoseItem(BodyPoseItem* item);
    void recordCurrentPose();
    bool loadPoseListFile(const std::string& filename);
    bool savePosesAsPoseListFile(const std::string& filename);

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    void restorePoses(BodyItem* bodyItem, const Listing* poseNodes);
    
    BodyItem* targetBodyItem_;
    int poseListArchiveMode_;
};

typedef ref_ptr<BodyPoseListItem> BodyPoseListItemPtr;

}

#endif
