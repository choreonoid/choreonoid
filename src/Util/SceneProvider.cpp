/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneProvider.h"
#include "SceneGraph.h"

using namespace cnoid;


SceneProvider::~SceneProvider()
{

}


SgNode* SceneProvider::getScene(SgCloneMap& cloneMap)
{
    return getScene()->cloneNode(cloneMap);
}
