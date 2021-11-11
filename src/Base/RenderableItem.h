#ifndef CNOID_BASE_RENDERABLE_ITEM_H
#define CNOID_BASE_RENDERABLE_ITEM_H

#include <cnoid/Signal>
#include "exportdecl.h"

namespace cnoid {

class SgNode;

class CNOID_EXPORT RenderableItem
{
public:
    RenderableItem();
    virtual ~RenderableItem();
    virtual SgNode* getScene() = 0;
    virtual bool isSceneSensitive();
    virtual void setSceneSensitive(bool on);
    virtual SignalProxy<void(bool on)> sigSceneSensitiveChanged();

    static void putSceneStatistics();

private:
    Signal<void(bool on)> sigSceneSensitiveChanged_;
    bool isSceneSensitive_;
};

typedef RenderableItem SceneProvider;
    
}

#endif
