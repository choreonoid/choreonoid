/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H
#define CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H

#include <cnoid/TimeSyncItemEngine>
#include <string>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class BodyItem;
class BodyMotionItem;
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
    virtual void onPlaybackStopped(double time, bool isStoppedManually) override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<BodyMotionEngine> BodyMotionEnginePtr;

}

#endif
