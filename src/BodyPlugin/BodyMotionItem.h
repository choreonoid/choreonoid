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

class CNOID_EXPORT BodyMotionItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    static void addExtraSeqItemFactory(
        const std::string& key, boost::function<AbstractSeqItemPtr(AbstractSeqPtr seq)> factory);

    BodyMotionItem();
    BodyMotionItem(BodyMotionPtr bodyMotion);
    ~BodyMotionItem();

    virtual AbstractMultiSeqPtr abstractMultiSeq();

    inline const BodyMotionPtr& motion() { return bodyMotion_; }

    inline MultiValueSeqItem* jointPosSeqItem() {
        return jointPosSeqItem_.get();
    }

    inline const MultiValueSeqPtr& jointPosSeq() {
        return bodyMotion_->jointPosSeq();
    }

    inline MultiSE3SeqItem* linkPosSeqItem() {
        return linkPosSeqItem_.get();
    }
            
    inline const MultiSE3SeqPtr& linkPosSeq() {
        return bodyMotion_->linkPosSeq();
    }

    int numExtraSeqItems() const;
    const std::string& extraSeqKey(int index) const;
    AbstractSeqItem* extraSeqItem(int index);
    const AbstractSeqItem* extraSeqItem(int index) const;
    SignalProxy<void()> sigExtraSeqItemsChanged();
    void updateExtraSeqItems();

    virtual void notifyUpdate();

protected:
    BodyMotionItem(const BodyMotionItem& org);

    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation);
    virtual ItemPtr doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

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
