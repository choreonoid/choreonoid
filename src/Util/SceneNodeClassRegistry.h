#ifndef CNOID_UTIL_SCENE_NODE_CLASS_REGISTRY
#define CNOID_UTIL_SCENE_NODE_CLASS_REGISTRY

#include "SceneGraph.h"
#include "HierarchicalClassRegistry.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneNodeClassRegistry : public HierarchicalClassRegistry<SgNode>
{
public:
    static SceneNodeClassRegistry& instance();

private:
    SceneNodeClassRegistry();
};

}

#endif
