#include "MultiDeviceStateSeqEngine.h"
#include "MultiDeviceStateSeqItem.h"
#include "BodyMotionEngine.h"
#include <cnoid/BodyItem>

using namespace std;
using namespace cnoid;


MultiDeviceStateSeqEngineCore::MultiDeviceStateSeqEngineCore(BodyItem* bodyItem)
{
    auto& devices = bodyItem->body()->devices();
    int numDevices = devices.size();
    deviceInfos.resize(numDevices);
    for(size_t i=0; i < numDevices; ++i){
        auto& info = deviceInfos[i];
        info.device = devices[i];
        auto& prevState = info.prevState;
        info.connection =
            info.device->sigStateChanged().connect(
                [this, &prevState](){ prevState.reset(); });
    }
}


void MultiDeviceStateSeqEngineCore::updateBodyDeviceStates(double time, const MultiDeviceStateSeq::Frame& states)
{
    int deviceIndex = 0;
    int n = deviceInfos.size();
    int m = std::min(n, static_cast<int>(states.size()));
    
    while(deviceIndex < m){
        DeviceState* state = states[deviceIndex];
        auto& info = deviceInfos[deviceIndex];
        if(state != info.prevState){
            info.device->copyStateFrom(*state);
            info.connection.block();
            info.device->notifyStateChange();
            info.connection.unblock();
            info.prevState = state;
        }
        info.device->notifyTimeChange(time);
        ++deviceIndex;
    }
    
    while(deviceIndex < n){
        deviceInfos[deviceIndex++].device->notifyTimeChange(time);
    }
}


void MultiDeviceStateSeqEngine::initializeClass(ExtensionManager* /* ext */)
{
    BodyMotionEngine::registerExtraSeqEngineFactory(
        MultiDeviceStateSeq::seqContentName(),
        [](BodyItem* bodyItem, AbstractSeqItem* seqItem) -> TimeSyncItemEngine* {
            MultiDeviceStateSeqEngine* engine = nullptr;
            if(auto multiDeviceStateSeqItem = dynamic_cast<MultiDeviceStateSeqItem*>(seqItem)){
                engine = new MultiDeviceStateSeqEngine(bodyItem, multiDeviceStateSeqItem);
            }
            return engine;
        });
}


MultiDeviceStateSeqEngine::MultiDeviceStateSeqEngine(BodyItem* bodyItem, MultiDeviceStateSeqItem* seqItem)
    : TimeSyncItemEngine(seqItem),
      seq(seqItem->seq()),
      core(bodyItem)
{
    seqItemConnection = seqItem->sigUpdated().connect([this](){ refresh(); });
}


bool MultiDeviceStateSeqEngine::onTimeChanged(double time)
{
    bool isValidTime = false;
    if(!seq->empty()){
        const int frame = seq->frameOfTime(time);
        core.updateBodyDeviceStates(time, seq->frame(seq->clampFrameIndex(frame)));
        if(frame < seq->numFrames()){
            isValidTime = true;
        }
    }
    return isValidTime;
}
