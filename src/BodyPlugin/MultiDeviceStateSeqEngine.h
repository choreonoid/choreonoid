#ifndef CNOID_BODYPLUGIN_MULTI_DEVICE_STATE_SEQ_ENGINE_H
#define CNOID_BODYPLUGIN_MULTI_DEVICE_STATE_SEQ_ENGINE_H

#include <cnoid/TimeSyncItemEngine>
#include <cnoid/MultiDeviceStateSeq>
#include <vector>
#include <memory>

namespace cnoid {

class BodyItem;
class MultiDeviceStateSeqItem;

class MultiDeviceStateSeqEngineCore
{
public:
    MultiDeviceStateSeqEngineCore(BodyItem* bodyItem);
    void updateBodyDeviceStates(double time, const MultiDeviceStateSeq::Frame& frame);

private:
    struct DeviceInfo {
        DevicePtr device;
        DeviceStatePtr prevState;
        ScopedConnection connection;
    };
    std::vector<DeviceInfo> deviceInfos;
};

class MultiDeviceStateSeqEngine : public TimeSyncItemEngine
{
public:
    static void initializeClass(ExtensionManager* ext);

    MultiDeviceStateSeqEngine(BodyItem* bodyItem, MultiDeviceStateSeqItem* seqItem);
    virtual bool onTimeChanged(double time) override;

private:
    std::shared_ptr<MultiDeviceStateSeq> seq;
    ScopedConnection seqItemConnection;
    MultiDeviceStateSeqEngineCore core;
};

}

#endif
