/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_PROVIDER_H
#define CNOID_UTIL_SCENE_PROVIDER_H

#include "exportdecl.h"

namespace cnoid {

class SgNode;
class CloneMap;

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
       This function returns a cloned scene instance.
       This function is implemented to return a clone of the scene instance returned by
       the getScene function. If the getScene function is implemented to return a cloned
       scene, it is necessary to override the clonseScene function so that it does not
       return any further clone of the cloned scene.
    */
    virtual SgNode* cloneScene(CloneMap& cloneMap);
};
    
}

#endif
