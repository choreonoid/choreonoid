#ifndef CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_ITEM_H
#define CNOID_POSE_SEQ_PLUGIN_POSE_SEQ_ITEM_H

#include "PoseSeq.h"
#include "PoseSeqInterpolator.h"
#include <cnoid/Item>
#include "exportdecl.h"

namespace cnoid {

class BodyItem;
class BodyMotionItem;

class CNOID_EXPORT PoseSeqItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);        
        
    PoseSeqItem();
    PoseSeqItem(const PoseSeqItem& org);
    ~PoseSeqItem();

    virtual bool setName(const std::string& name) override;

    BodyItem* targetBodyItem();
    PoseSeq* poseSeq();
    PoseSeqInterpolatorPtr interpolator();
    BodyMotionItem* bodyMotionItem();
    double barLength() const;

    typedef std::vector<PoseSeq::iterator> PoseSeqIteratorList;
    const PoseSeqIteratorList& selectedPoses() const;
    //! \return true if any selected item is actually cleared.
    bool clearPoseSelection(bool doNotify = false);
    void selectPose(PoseSeq::iterator pose, bool doNotify = false, bool doSort = true);
    //! \return true if the elemnt is actually unselected from the selected state.
    bool deselectPose(PoseSeq::iterator pose, bool doNotify = false);
    void selectAllPoses(bool doNotify = false);
    bool checkSelected(PoseSeq::iterator pose) const;
    void notifyPoseSelectionChange();

    /**
       \note When a selected pose is removed from the sequence with the PoseSeq::erase function,
       the pose is also removed from the selection and sigPoseSelectionChanged is emitted.
    */
    SignalProxy<void(const std::vector<PoseSeq::iterator>& selected)> sigPoseSelectionChanged();

    bool updateInterpolation();
    bool updateTrajectory(bool putMessages = false);

    void beginEditing();
    bool endEditing(bool actuallyModified = true);
    void clearEditHistory();
            
    bool undo();
    bool redo();

    /**
       temporary treatment.
    */
    bool updatePosesWithBalancedTrajectories(std::ostream& os);

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
