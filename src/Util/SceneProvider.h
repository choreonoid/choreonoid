/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_PROVIDER_H
#define CNOID_UTIL_SCENE_PROVIDER_H

namespace cnoid {

class SgNode;

class SceneProvider
{
public:
    virtual SgNode* scene() = 0;
};
    
}

#endif
