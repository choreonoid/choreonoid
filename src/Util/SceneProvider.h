/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_PROVIDER_H_INCLUDED
#define CNOID_UTIL_SCENE_PROVIDER_H_INCLUDED

namespace cnoid {

class SgNode;

class CNOID_EXPORT SceneProvider
{
public:
    virtual SgNode* scene() = 0;
};
}

#endif
