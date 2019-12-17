#ifndef CNOID_BASE_RENDERABLE_ITEM_H
#define CNOID_BASE_RENDERABLE_ITEM_H

namespace cnoid {

class SgNode;

class RenderableItem
{
public:
    virtual SgNode* getScene() = 0;
};

typedef RenderableItem SceneProvider;
    
}

#endif
