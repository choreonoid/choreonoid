/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POINT_SET_UTIL_H_INCLUDED
#define CNOID_UTIL_POINT_SET_UTIL_H_INCLUDED

#include <cnoid/SceneShape>
#include "exportdecl.h"

namespace cnoid {
CNOID_EXPORT void loadPCD(SgPointSet* out_pointSet, const std::string& filename);
CNOID_EXPORT void savePCD(SgPointSet* pointSet, const std::string& filename, const Affine3d& viewpoint = Affine3d::Identity());
}

#endif
