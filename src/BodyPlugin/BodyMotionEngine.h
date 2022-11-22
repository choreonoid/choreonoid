#ifndef CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H
#define CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H

#include <cnoid/TimeSyncItemEngine>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class Body;
class BodyItem;
class BodyMotionItem;
class BodyPositionSeqFrame;
class AbstractSeqItem;

class CNOID_EXPORT BodyMotionEngine : public TimeSyncItemEngine
{
public:
    static void initializeClass(ExtensionManager* ext);

    static void addExtraSeqEngineFactory(
        const std::string& key, std::function<TimeSyncItemEngine*(BodyItem* bodyItem, AbstractSeqItem* seqItem)> factory);

    BodyMotionEngine(BodyItem* bodyItem, BodyMotionItem* motionItem);
    virtual ~BodyMotionEngine();

    BodyItem* bodyItem();
    BodyMotionItem* motionItem();
        
    virtual void onPlaybackStarted(double time) override;
    virtual bool onTimeChanged(double time) override;
    virtual double onPlaybackStopped(double time, bool isStoppedManually) override;

    //! \return true if the forward kinematics from the root link must be processed.
    static bool updateBodyPositionWithBodyPositionSeqFrame(
        Body* body, const BodyPositionSeqFrame& frame);
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodyMotionEngine> BodyMotionEnginePtr;

}

#endif
