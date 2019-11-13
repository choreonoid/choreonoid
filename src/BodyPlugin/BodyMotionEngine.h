/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H
#define CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H

#include <cnoid/TimeSyncItemEngine>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class BodyItem;
class BodyMotionItem;
class AbstractSeqItem;
class BodyMotionEngineImpl;


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
        
    virtual bool onTimeChanged(double time);

private:
    BodyMotionEngineImpl* impl;
};

typedef ref_ptr<BodyMotionEngine> BodyMotionEnginePtr;

}

#endif
