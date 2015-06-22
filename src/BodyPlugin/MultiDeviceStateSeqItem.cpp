/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiDeviceStateSeqItem.h"
#include "BodyItem.h"
#include "BodyMotionItem.h"
#include "BodyMotionEngine.h"
#include <cnoid/ItemManager>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

AbstractSeqItem* createMultiDeviceStateSeqItem(AbstractSeqPtr seq)
{
    MultiDeviceStateSeqPtr dseq = dynamic_pointer_cast<MultiDeviceStateSeq>(seq);
    return dseq ? new MultiDeviceStateSeqItem(dseq) : 0;
}


class MultiDeviceStateSeqEngine : public TimeSyncItemEngine
{
    MultiDeviceStateSeqPtr seq;
    BodyPtr body;
    vector<DeviceStatePtr> prevStates;

public:
        
    MultiDeviceStateSeqEngine(MultiDeviceStateSeqItem* seqItem, BodyItem* bodyItem)
        : seq(seqItem->seq()), body(bodyItem->body()) {
        seqItem->sigUpdated().connect(boost::bind(&TimeSyncItemEngine::notifyUpdate, this));
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
                const DeviceStatePtr& state = states[i];
                if(state != prevStates[i]){
                    const DevicePtr& device = devices[i];
                    device->copyStateFrom(*state);
                    device->notifyStateChange();
                    prevStates[i] = state;
                }
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
    : seq_(boost::make_shared<MultiDeviceStateSeq>())
{

}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(MultiDeviceStateSeqPtr seq)
    : seq_(seq)
{
    setName(seq->seqContentName());
}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(const MultiDeviceStateSeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(boost::make_shared<MultiDeviceStateSeq>(*org.seq_))
{

}


MultiDeviceStateSeqItem::~MultiDeviceStateSeqItem()
{

}


AbstractMultiSeqPtr MultiDeviceStateSeqItem::abstractMultiSeq()
{
    return seq_;
}


ItemPtr MultiDeviceStateSeqItem::doDuplicate() const
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
