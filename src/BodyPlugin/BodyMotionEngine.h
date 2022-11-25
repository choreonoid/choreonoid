#ifndef CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H
#define CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H

#include <cnoid/TimeSyncItemEngine>
#include <cnoid/BodyItem>
#include <cnoid/BodyMotionItem>
#include <cnoid/BodyPositionSeq>
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
    BodyItem* bodyItem() { return bodyItem_; }
    void updateBodyPosition(const BodyPositionSeqFrame& frame);

private:
    BodyItemPtr bodyItem_;
    BodyPtr body;

    bool updateBodyPosition_(const BodyPositionSeqFrame& frame);
    bool updateSingleBodyPosition(Body* body, BodyPositionSeqFrameBlock frameBlock);
    void updateBodyVelocity(const BodyPositionSeqFrame& prevFrame, double timeStep);
    friend class BodyMotionEngine;
};

class CNOID_EXPORT BodyMotionEngine : public TimeSyncItemEngine
{
public:
    static void initializeClass(ExtensionManager* ext);

    static void addExtraSeqEngineFactory(
        const std::string& key, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory);

    BodyMotionEngine(BodyItem* bodyItem, BodyMotionItem* motionItem);

    BodyItem* bodyItem() { return core.bodyItem_; }
    BodyMotionItem* motionItem() { return motionItem_; }
        
    virtual void onPlaybackStarted(double time) override;
    virtual bool onTimeChanged(double time) override;
    virtual double onPlaybackStopped(double time, bool isStoppedManually) override;

private:
    BodyMotionEngineCore core;
    BodyMotionItemPtr motionItem_;
    std::shared_ptr<BodyPositionSeq> positionSeq;
    std::vector<TimeSyncItemEnginePtr> extraSeqEngines;
    ScopedConnectionSet connections;

    void updateExtraSeqEngines();
};

typedef ref_ptr<BodyMotionEngine> BodyMotionEnginePtr;

}

#endif
