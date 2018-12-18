/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_POSESEQ_PLUGIN_POSE_SEQ_ITEM_H
#define CNOID_POSESEQ_PLUGIN_POSE_SEQ_ITEM_H

#include "PoseSeq.h"
#include "PoseSeqInterpolator.h"
#include <cnoid/Item>
#include <cnoid/BodyMotionItem>
#include <set>
#include <deque>
#include "exportdecl.h"

namespace cnoid {

class TimeBar;
class BodyItem;
class BodyMotionGenerationBar;

class CNOID_EXPORT PoseSeqItem : public Item
{
public:
    static void initializeClass(ExtensionManager* ext);        
        
    PoseSeqItem();
    PoseSeqItem(const PoseSeqItem& org);
    ~PoseSeqItem();
            
    virtual void setName(const std::string& name) override;

    PoseSeqPtr poseSeq() { return seq; }
    PoseSeqInterpolatorPtr interpolator(){ return interpolator_; }
    BodyMotionItem* bodyMotionItem(){ return bodyMotionItem_; }

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

    double barLength() const { return barLength_; }

protected:

    virtual Item* doDuplicate() const override;
    virtual void onPositionChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    BodyItem* ownerBodyItem;
    PoseSeqPtr seq;
    PoseSeqInterpolatorPtr interpolator_;
    BodyMotionItemPtr bodyMotionItem_;
    Connection sigInterpolationParametersChangedConnection;

    ConnectionSet editConnections;

    struct EditHistory {
        /*
          Unify these containers into one which contains elements
          in the operated orders and restore them in the same order
          when undo or redo is carried out.
        */
        PoseSeqPtr removed;
        PoseSeqPtr added;
        EditHistory(){
            removed = new PoseSeq();
            added = new PoseSeq();
        }
        bool empty(){
            return removed->empty() && added->empty();
        }
        void clear(){
            if(!empty()){
                removed = new PoseSeq();
                added = new PoseSeq();
            }
        }
    };

    struct PoseIterComp {
        bool operator()(const PoseSeq::iterator it1, const PoseSeq::iterator it2) const {
            return &(*it1) < &(*it2);
        }
    };
    std::set<PoseSeq::iterator, PoseIterComp> inserted;
    std::set<PoseSeq::iterator, PoseIterComp> modified;
            
    double modifyingPoseTime;
    double modifyingPoseTTime;
    PoseUnitPtr modifyingPoseUnitOrg;
    PoseSeq::iterator modifyingPoseIter;

    std::deque<EditHistory> editHistories;
    EditHistory newHistory;
    int currentHistory;

    BodyMotionGenerationBar* generationBar;
    TimeBar* timeBar;

    bool isSelectedPoseMoving;

    double barLength_;

    void init();
    void convert(BodyPtr orgBody);
    bool convertSub(BodyPtr orgBody, const Mapping& convInfo);
    void updateInterpolationParameters();
    void onInserted(PoseSeq::iterator p, bool isMoving);
    void onRemoving(PoseSeq::iterator p, bool isMoving);
    void onModifying(PoseSeq::iterator p);
    void onModified(PoseSeq::iterator p);
    PoseSeq::iterator removeSameElement(PoseSeq::iterator current, PoseSeq::iterator p);
};

typedef ref_ptr<PoseSeqItem> PoseSeqItemPtr;
}

#endif
