/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H_INCLUDED
#define CNOID_BODYPLUGIN_BODY_MOTION_ENGINE_H_INCLUDED

#include <cnoid/TimeSyncItemEngine>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class BodyMotionEngineImpl;

class BodyItem;
typedef ref_ptr<BodyItem> BodyItemPtr;

class BodyMotionItem;
typedef ref_ptr<BodyMotionItem> BodyMotionItemPtr;

class AbstractSeqItem;
typedef ref_ptr<AbstractSeqItem> AbstractSeqItemPtr;

class CNOID_EXPORT BodyMotionEngine : public TimeSyncItemEngine
{
public:
    static void initialize(ExtensionManager* ext);

    static void addExtraSeqEngineFactory(
        const std::string& key, boost::function<TimeSyncItemEnginePtr(BodyItemPtr bodyItem, AbstractSeqItemPtr seqItem)> factory);

    BodyMotionEngine(BodyItemPtr bodyItem, BodyMotionItemPtr motionItem);
    virtual ~BodyMotionEngine();

    BodyItem* bodyItem();
    BodyMotionItem* motionItem();
        
    virtual bool onTimeChanged(double time);

private:
    BodyMotionEngineImpl* impl;
};

typedef boost::shared_ptr<BodyMotionEngine> BodyMotionEnginePtr;
}

#endif
