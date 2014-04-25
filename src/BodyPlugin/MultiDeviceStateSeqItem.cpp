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
using namespace boost;
using namespace cnoid;


namespace {

AbstractSeqItemPtr createMultiDeviceStateSeqItem(AbstractSeqPtr seq)
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
        
    MultiDeviceStateSeqEngine(MultiDeviceStateSeqItemPtr seqItem, BodyItemPtr bodyItem)
        : seq(seqItem->seq()), body(bodyItem->body())
        {
            seqItem->sigUpdated().connect(bind(&TimeSyncItemEngine::notifyUpdate, this));
        }

    virtual bool onTimeChanged(double time)
        {
            bool isValidTime = false;
            if(!seq->empty()){
                const DeviceList<>& devices = body->devices();
                MultiDeviceStateSeq::Frame states = seq->frame(seq->clampFrameIndex(seq->frameOfTime(time), isValidTime));
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


TimeSyncItemEnginePtr createMultiDeviceStateSeqEngine(BodyItemPtr bodyItem, AbstractSeqItemPtr seqItem)
{
    if(MultiDeviceStateSeqItemPtr item = dynamic_pointer_cast<MultiDeviceStateSeqItem>(seqItem)){
        return make_shared<MultiDeviceStateSeqEngine>(item, bodyItem);
    }
    return TimeSyncItemEnginePtr();
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
    : seq_(make_shared<MultiDeviceStateSeq>())
{

}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(MultiDeviceStateSeqPtr seq)
    : seq_(seq)
{
    setName(seq->seqContentName());
}


MultiDeviceStateSeqItem::MultiDeviceStateSeqItem(const MultiDeviceStateSeqItem& org)
    : AbstractMultiSeqItem(org),
      seq_(make_shared<MultiDeviceStateSeq>(*org.seq_))
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
