/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_PROVIDER_H
#define CNOID_UTIL_SCENE_PROVIDER_H

#include "exportdecl.h"

namespace cnoid {

class SgNode;
class SgCloneMap;

class CNOID_EXPORT SceneProvider
{
public:

    virtual ~SceneProvider();

    /**
       If the scene has some state that affect the rendering,
       the scene node should be cloned for each call of this function.
       Otherwise, the same instance can be returned.
    */
    virtual SgNode* getScene() = 0;

    /**
       Whether the getScene function returns the shared instance or a cloned instance,
       this function must always returns a cloned instance.
       If the getScene function returns a clone instance, this function should be overridden
       to avoid redundant cloning.
    */
    virtual SgNode* getScene(SgCloneMap& cloneMap);
};
    
}

#endif
