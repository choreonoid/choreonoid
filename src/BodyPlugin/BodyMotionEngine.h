#ifndef CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H
#define CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H

#include <cnoid/TimeSyncItemEngine>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/BodyStateSeq>
#include <cnoid/Device>
#include <cnoid/ConnectionSet>
#include <memory>
#include <vector>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;

class BodyMotionEngineCore
{
public:
    BodyMotionEngineCore(BodyItem* bodyItem);
    BodyItem* bodyItem() { return bodyItemRef.lock(); }
    void updateBodyState(double time, const BodyState& state);

private:
    weak_ref_ptr<BodyItem> bodyItemRef;

    struct DeviceInfo {
        DevicePtr device;
        DeviceStatePtr prevState;
        ScopedConnection connection;
    };
    std::vector<DeviceInfo> deviceInfos;
    
    bool updateBodyState_(double time, Body* body, const BodyState& state);
    bool updateSingleBodyState(double time, Body* body, BodyStateBlock stateBlock, bool isMainBody);
    void updateBodyVelocity(Body* body, const BodyState& prevState, double timeStep);
    friend class BodyMotionEngine;
};

class CNOID_EXPORT BodyMotionEngine : public TimeSyncItemEngine
{
public:
    static void initializeClass(ExtensionManager* ext);

    static void registerExtraSeqEngineFactory(
        const std::string& contentName, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory);
    [[deprecated("Use registerExtraSeqEngineFactory")]]
    static void addExtraSeqEngineFactory(
        const std::string& contentName, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory);

    BodyMotionEngine(BodyItem* bodyItem, BodyMotionItem* motionItem);

    BodyItem* bodyItem() { return core.bodyItem(); }
    BodyMotionItem* motionItem() { return motionItem_; }
        
    virtual void onPlaybackStarted(double time) override;
    virtual bool onTimeChanged(double time) override;
    virtual double onPlaybackStopped(double time, bool isStoppedManually) override;

private:
    BodyMotionEngineCore core;
    BodyMotionItem* motionItem_;
    std::shared_ptr<BodyStateSeq> stateSeq;
    std::vector<TimeSyncItemEnginePtr> extraSeqEngines;
    ScopedConnectionSet connections;

    void updateExtraSeqEngines();
};

typedef ref_ptr<BodyMotionEngine> BodyMotionEnginePtr;

}

#endif
