#ifndef CNOID_BODY_PLUGIN_MULTI_DEVICE_STATE_SEQ_ITEM_H
#define CNOID_BODY_PLUGIN_MULTI_DEVICE_STATE_SEQ_ITEM_H

#include <cnoid/MultiDeviceStateSeq>
#include <cnoid/AbstractSeqItem>
#include "exportdecl.h"

namespace cnoid {

class MultiDeviceStateSeqItem : public AbstractMultiSeqItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    MultiDeviceStateSeqItem();
    MultiDeviceStateSeqItem(std::shared_ptr<MultiDeviceStateSeq> seq);
    virtual ~MultiDeviceStateSeqItem();
    virtual std::shared_ptr<AbstractMultiSeq> abstractMultiSeq() override;
    std::shared_ptr<MultiDeviceStateSeq> seq() { return seq_; }

protected:
    MultiDeviceStateSeqItem(const MultiDeviceStateSeqItem& org);
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

private:
    std::shared_ptr<MultiDeviceStateSeq> seq_;
};

typedef ref_ptr<MultiDeviceStateSeqItem> MultiDeviceStateSeqItemPtr;
}

#endif
