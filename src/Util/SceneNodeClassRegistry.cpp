#include "SceneNodeClassRegistry.h"

using namespace cnoid;

namespace cnoid {

SceneNodeClassRegistry SceneNodeClassRegistry::instance_;

}


SceneNodeClassRegistry::SceneNodeClassRegistry()
{
    reserve(50);
}
