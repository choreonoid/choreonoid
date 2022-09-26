/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_ITEM_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_ITEM_H

#include "PoseSeqInterpolator.h"
#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class PoseSeq;
class BodyMotionItem;

class CNOID_EXPORT PoseSeqItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);        
        
    PoseSeqItem();
    PoseSeqItem(const PoseSeqItem& org);
    ~PoseSeqItem();
            
    virtual bool setName(const std::string& name) override;

    PoseSeq* poseSeq();
    PoseSeqInterpolatorPtr interpolator();
    BodyMotionItem* bodyMotionItem();
    double barLength() const;

    virtual bool updateInterpolation();
    virtual bool updateTrajectory(bool putMessages = false);

    void beginEditing();
    bool endEditing(bool actuallyModified = true);
    void clearEditHistory();
            
    bool undo();
    bool redo();

    /**
       temporary treatment.
    */
    bool updateKeyPosesWithBalancedTrajectories(std::ostream& os);

protected:
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<PoseSeqItem> PoseSeqItemPtr;

}

#endif
