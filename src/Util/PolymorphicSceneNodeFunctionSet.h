#ifndef CNOID_UTIL_POLYMORPHIC_SCENE_NODE_FUNCTION_SET_H
#define CNOID_UTIL_POLYMORPHIC_SCENE_NODE_FUNCTION_SET_H

#include "SceneGraph.h"
#include "PolymorphicFunctionSet.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PolymorphicSceneNodeFunctionSet : public PolymorphicFunctionSet<SgNode>
{
public:
    PolymorphicSceneNodeFunctionSet();
};

}

#endif
