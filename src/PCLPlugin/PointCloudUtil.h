/**
   @file
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H
#define CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H

#include <cnoid/SceneShape>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT SgMesh* createSurfaceMesh(SgPointSet* pointSet);
CNOID_EXPORT bool alignPointCloud(SgPointSet* target, SgPointSet* source, Affine3& io_T);

}

#endif
