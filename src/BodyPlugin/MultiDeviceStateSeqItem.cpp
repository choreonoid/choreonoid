/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiDeviceStateSeqItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "BodyMotionEngine.h"
#include <cnoid/ItemManager>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

AbstractSeqItem* createMultiDeviceStateSeqItem(std::shared_ptr<AbstractSeq> seq)
{
    auto dseq = dynamic_pointer_cast<MultiDeviceStateSeq>(seq);
    if(dseq){
        auto item = new MultiDeviceStateSeqItem(dseq);
        item->setName("Devices");
        return item;
    }
    return nullptr;
}


class MultiDeviceStateSeqEngine : public TimeSyncItemEngine
{
    shared_ptr<MultiDeviceStateSeq> seq;
    BodyPtr body;
    vector<DeviceStatePtr> prevStates;

public:
        
    MultiDeviceStateSeqEngine(MultiDeviceStateSeqItem* seqItem, BodyItem* bodyItem)
        : seq(seqItem->seq()), body(bodyItem->body()) {
        seqItem->sigUpdated().connect(std::bind(&TimeSyncItemEngine::notifyUpdate, this));
    }

    virtual bool onTimeChanged(double time){
        bool isValidTime = false;
        if(!seq->empty()){
            const DeviceList<>& devices = body->devices();
            const int frame = seq->frameOfTime(time);
            isValidTime = (frame < seq->numFrames());
            MultiDeviceStateSeq::Frame states = seq->frame(seq->clampFrameIndex(frame));
            const int n = std::min((int)devices.size(), states.size());
            prevStates.resize(n);
            for(int i=0; i < n; ++i){
                DeviceState* state = states[i];
                Device* device = devices[i];
                if(state != prevStates[i]){
                    device->copyStateFrom(*state);
                    device->notifyStateChange();
                    prevStates[i] = state;
                }
                device->notifyTimeChange(time);
            }
        }
        return isValidTime;
    }
};


TimeSyncItemEngine* createMultiDeviceStateSeqEngine(BodyItem* bodyItem, AbstractSeqItem* seqItem)
{
    if(MultiDeviceStateSeqItem* item = dynamic_cast<MultiDeviceStateSeqItem*>(seqItem)){
        return new MultiDeviceStateSeqEngine(item, bodyItem);
    }
    return 0;
}

}


void MultiDeviceStateSeqItem::initializeClass(ExtensionManager* ext)
{
    static bool initialized = false;

    if(!initialized){
        
        ext->itemManager().registerClass<MultiDeviceStateSeqItem>(N_("MultiDeviceStateSeqItem"));
        
        BodyMotionItem::addExtraSeqItemFactory(MultiDeviceStateSeq::key(), createMultiDeviceStateSeqItem);
        BodyMotionEngine::addExtraSeqEngineFactory(MultiDeviceStateSeq::key(), createMultiDeviceStateSeqEngine);
        
        initialized = true;
    }
}

        
MultiDeviceStateSeqItem::MultiDeviceStateSeqItem()
    : seq_(std::make_shared<MultiDeviceStateSeq>())
{

}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(std::shared_ptr<MultiDeviceStateSeq> seq)
    : seq_(seq)
{
    setName(seq->seqContentName());
}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(const MultiDeviceStateSeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(std::make_shared<MultiDeviceStateSeq>(*org.seq_))
{

}


MultiDeviceStateSeqItem::~MultiDeviceStateSeqItem()
{

}


std::shared_ptr<AbstractMultiSeq> MultiDeviceStateSeqItem::abstractMultiSeq()
{
    return seq_;
}


Item* MultiDeviceStateSeqItem::doDuplicate() const
{
    return new MultiDeviceStateSeqItem(*this);
}


bool MultiDeviceStateSeqItem::store(Archive& archive)
{
    return false;
}


bool MultiDeviceStateSeqItem::restore(const Archive& archive)
{
    return false;
}
