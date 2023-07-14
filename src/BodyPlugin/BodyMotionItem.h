#ifndef CNOID_BODY_PLUGIN_BODY_MOTION_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_MOTION_ITEM_H

#include <cnoid/AbstractSeqItem>
#include <cnoid/BodyMotion>
#include <memory>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT BodyMotionItem : public AbstractSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    static void registerExtraSeqType(
        const std::string& typeName, std::function<AbstractSeqItem*(std::shared_ptr<AbstractSeq> seq)> itemFactory);

    static void registerExtraSeqContent(
        const std::string& contentName, std::function<AbstractSeqItem*(std::shared_ptr<AbstractSeq> seq)> itemFactory);

    [[deprecated("Use registerExtraSeqContent")]]
    static void addExtraSeqItemFactory(
        const std::string& contentName, std::function<AbstractSeqItem*(std::shared_ptr<AbstractSeq> seq)> itemFactory) {
        registerExtraSeqContent(contentName, itemFactory);
    }

    BodyMotionItem();
    BodyMotionItem(std::shared_ptr<BodyMotion> bodyMotion);
    ~BodyMotionItem();

    virtual std::shared_ptr<AbstractSeq> abstractSeq() override;

    std::shared_ptr<BodyMotion> motion() { return bodyMotion_; }
    std::shared_ptr<const BodyMotion> motion() const { return bodyMotion_; }

    int numExtraSeqItems() const;
    const std::string& extraSeqContentName(int index) const;
    [[deprecated("Use extraSeqContentName.")]]
    const std::string& extraSeqKey(int index) const { return extraSeqContentName(index); }
    AbstractSeqItem* extraSeqItem(int index);
    const AbstractSeqItem* extraSeqItem(int index) const;
    SignalProxy<void()> sigExtraSeqItemsChanged();
    void updateExtraSeqItems();

    [[deprecated("Use motion()->jointPosSeq()")]]
    std::shared_ptr<MultiValueSeq> jointPosSeq() {
        return bodyMotion_->jointPosSeq();
    }

    bool isBodyJointVelocityUpdateEnabled() const {
        return isBodyJointVelocityUpdateEnabled_;
    }
    void setBodyJointVelocityUpdateEnabled(bool on) {
        isBodyJointVelocityUpdateEnabled_ = on;
    }

    virtual void notifyUpdate() override;

protected:
    BodyMotionItem(const BodyMotionItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual bool onChildItemAboutToBeAdded(Item* childItem, bool isManualOperation) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    std::shared_ptr<BodyMotion> bodyMotion_;
    bool isBodyJointVelocityUpdateEnabled_;

    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodyMotionItem> BodyMotionItemPtr;

}

#endif
