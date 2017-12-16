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

class BodyMotionItemImpl;

class CNOID_EXPORT BodyMotionItem : public AbstractSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    static void addExtraSeqItemFactory(
        const std::string& key, std::function<AbstractSeqItem*(AbstractSeqPtr seq)> factory);

    BodyMotionItem();
    BodyMotionItem(BodyMotionPtr bodyMotion);
    BodyMotionItem(const BodyMotionItem& org);
    ~BodyMotionItem();

    virtual AbstractSeqPtr abstractSeq() override;

    BodyMotionPtr motion() { return bodyMotion_; }
    ConstBodyMotionPtr motion() const { return bodyMotion_; }

    MultiValueSeqItem* jointPosSeqItem() {
        return jointPosSeqItem_;
    }

    const MultiValueSeqItem* jointPosSeqItem() const {
        return jointPosSeqItem_;
    }

    MultiValueSeqPtr jointPosSeq() {
        return bodyMotion_->jointPosSeq();
    }

    MultiSE3SeqItem* linkPosSeqItem() {
        return linkPosSeqItem_;
    }

    const MultiSE3SeqItem* linkPosSeqItem() const {
        return linkPosSeqItem_;
    }
    
    MultiSE3SeqPtr linkPosSeq() {
        return bodyMotion_->linkPosSeq();
    }

    int numExtraSeqItems() const;
    const std::string& extraSeqKey(int index) const;
    AbstractSeqItem* extraSeqItem(int index);
    const AbstractSeqItem* extraSeqItem(int index) const;
    SignalProxy<void()> sigExtraSeqItemsChanged();
    void updateExtraSeqItems();

    virtual void notifyUpdate() override;

protected:
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation) override;
    virtual Item* doDuplicate() const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    BodyMotionPtr bodyMotion_;
    MultiValueSeqItemPtr jointPosSeqItem_;
    MultiSE3SeqItemPtr linkPosSeqItem_;

    BodyMotionItemImpl* impl;
    friend class BodyMotionItemImpl;
};

typedef ref_ptr<BodyMotionItem> BodyMotionItemPtr;

}

#endif
