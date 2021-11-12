/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_MOTION_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_MOTION_ITEM_H

#include <cnoid/BodyMotion>
#include <cnoid/MultiValueSeqItem>
#include <cnoid/MultiSE3SeqItem>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyMotionItem : public AbstractSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    static void addExtraSeqItemFactory(
        const std::string& key, std::function<AbstractSeqItem*(std::shared_ptr<AbstractSeq> seq)> factory);

    BodyMotionItem();
    BodyMotionItem(std::shared_ptr<BodyMotion> bodyMotion);
    BodyMotionItem(const BodyMotionItem& org);
    ~BodyMotionItem();

    virtual std::shared_ptr<AbstractSeq> abstractSeq() override;

    std::shared_ptr<BodyMotion> motion() { return bodyMotion_; }
    std::shared_ptr<const BodyMotion> motion() const { return bodyMotion_; }

    MultiValueSeqItem* jointPosSeqItem() {
        return jointPosSeqItem_;
    }

    const MultiValueSeqItem* jointPosSeqItem() const {
        return jointPosSeqItem_;
    }

    std::shared_ptr<MultiValueSeq> jointPosSeq() {
        return bodyMotion_->jointPosSeq();
    }

    MultiSE3SeqItem* linkPosSeqItem() {
        return linkPosSeqItem_;
    }

    const MultiSE3SeqItem* linkPosSeqItem() const {
        return linkPosSeqItem_;
    }
    
    std::shared_ptr<MultiSE3Seq> linkPosSeq() {
        return bodyMotion_->linkPosSeq();
    }

    int numExtraSeqItems() const;
    const std::string& extraSeqKey(int index) const;
    AbstractSeqItem* extraSeqItem(int index);
    const AbstractSeqItem* extraSeqItem(int index) const;
    SignalProxy<void()> sigExtraSeqItemsChanged();
    void updateExtraSeqItems();

    bool isBodyJointVelocityUpdateEnabled() const {
        return isBodyJointVelocityUpdateEnabled_;
    }
    void setBodyJointVelocityUpdateEnabled(bool on) {
        isBodyJointVelocityUpdateEnabled_ = on;
    }

    virtual void notifyUpdate() override;

protected:
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation) override;
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    std::shared_ptr<BodyMotion> bodyMotion_;
    MultiValueSeqItemPtr jointPosSeqItem_;
    MultiSE3SeqItemPtr linkPosSeqItem_;

    class Impl;
    Impl* impl;

    bool isBodyJointVelocityUpdateEnabled_;
};

typedef ref_ptr<BodyMotionItem> BodyMotionItemPtr;

}

#endif
