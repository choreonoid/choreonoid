/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POINT_SET_UTIL_H
#define CNOID_UTIL_POINT_SET_UTIL_H

#include <cnoid/SceneDrawables>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT void loadPCD(SgPointSet* out_pointSet, const std::string& filename);
CNOID_EXPORT void savePCD(SgPointSet* pointSet, const std::string& filename, const Isometry3& viewpoint = Isometry3::Identity());

}

#endif
