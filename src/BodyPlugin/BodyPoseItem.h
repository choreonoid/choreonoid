#ifndef CNOID_BODY_PLUGIN_BODY_POSE_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_POSE_ITEM_H

#include <cnoid/Item>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class BodyPoseListItem;
class Body;
class Mapping;

class CNOID_EXPORT BodyPoseItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    BodyPoseItem();
    BodyPoseItem(const BodyPoseItem& org);
    virtual std::string displayName() const override;

    BodyPoseListItem* poseListItem() { return poseListItem_; }
    const BodyPoseListItem* poseListItem() const { return poseListItem_; }
    BodyItem* targetBodyItem() const;
    bool fetchBodyPose();
    bool applyBodyPose();
    bool readBodyPose(const Mapping* archive, const Body* body, bool doReadMetaData);
    bool writeBodyPose(Mapping* archive, bool doWriteMetaData) const;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual bool onNewTreePositionCheck(bool isManualOperation, std::function<void()>& out_callbackWhenAdded) override;
    virtual void onDoubleClicked() override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    void restorePose(const Mapping* archive, BodyItem* bodyItem);
    
    std::vector<double> jointDisplacements;
    BodyPoseListItem* poseListItem_;
};

typedef ref_ptr<BodyPoseItem> BodyPoseItemPtr;

}

#endif
