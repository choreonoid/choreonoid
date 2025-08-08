#include "SceneNodeClassRegistry.h"

using namespace cnoid;


SceneNodeClassRegistry& SceneNodeClassRegistry::instance()
{
    // Use function-local static to avoid static initialization order fiasco
    static SceneNodeClassRegistry instance_;
    return instance_;
}


SceneNodeClassRegistry::SceneNodeClassRegistry()
    : HierarchicalClassRegistry<SgNode>("SgNode")
{
    reserve(50);
}
