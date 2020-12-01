/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_UTIL_H
#define CNOID_UTIL_SCENE_UTIL_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT Affine3 calcTotalTransform(const SgNodePath& path);
CNOID_EXPORT Affine3 calcTotalTransform(const SgNodePath& path, const SgNode* targetNode);
CNOID_EXPORT Affine3 calcTotalTransform(SgNodePath::const_iterator begin, SgNodePath::const_iterator end);

CNOID_EXPORT Isometry3 calcRelativePosition(const SgNodePath& path, const SgNode* targetNode);
CNOID_EXPORT Isometry3 calcRelativePosition(SgNodePath::const_iterator begin, SgNodePath::const_iterator end);

CNOID_EXPORT int makeTransparent(SgNode* topNode, float transparency, CloneMap& cloneMap, bool doKeepOrgTransparency = true);

}

#endif
