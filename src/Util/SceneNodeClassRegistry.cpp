#include "SceneNodeClassRegistry.h"

using namespace cnoid;


SceneNodeClassRegistry& SceneNodeClassRegistry::instance()
{
    static SceneNodeClassRegistry registry;
    return registry;
}


SceneNodeClassRegistry::SceneNodeClassRegistry()
{

}
