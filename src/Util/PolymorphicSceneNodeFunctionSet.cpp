#include "PolymorphicSceneNodeFunctionSet.h"
#include "SceneNodeClassRegistry.h"

using namespace cnoid;


PolymorphicSceneNodeFunctionSet::PolymorphicSceneNodeFunctionSet()
    : PolymorphicFunctionSet<SgNode>(SceneNodeClassRegistry::instance())
{

}
