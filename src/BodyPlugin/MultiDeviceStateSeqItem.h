/**
   @file
   @author Shin'ichiro Nakaoka
*/

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
    MultiDeviceStateSeqItem(MultiDeviceStateSeqPtr seq);
    MultiDeviceStateSeqItem(const MultiDeviceStateSeqItem& org);
    virtual ~MultiDeviceStateSeqItem();
    virtual AbstractMultiSeqPtr abstractMultiSeq();
    MultiDeviceStateSeqPtr seq() { return seq_; }

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);

private:
    MultiDeviceStateSeqPtr seq_;
};

typedef ref_ptr<MultiDeviceStateSeqItem> MultiDeviceStateSeqItemPtr;
}

#endif
