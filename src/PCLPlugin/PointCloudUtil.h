#ifndef CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H
#define CNOID_PCL_PLUGIN_POINT_CLOUD_UTIL_H

#include <cnoid/SceneDrawables>
#include <cnoid/stdx/optional>
#include "exportdecl.h"

namespace cnoid {

CNOID_EXPORT SgMesh* applyPCLGreedyProjectionTriangulation(
    SgPointSet* pointSet,
    int normalEstimationKSearch,
    double normalEstimationSearchRadius,
    int maxNearestNeighbors, double mu,
    double searchRadius,
    double minAngle, double maxAngle,
    double maxSurfaceAngle, bool normalConsistency);

CNOID_EXPORT stdx::optional<double> alignPointCloud
(SgPointSet* target, SgPointSet* source, Affine3& io_T,
 double maxCorrespondenceDistance, int maxIterations, double epsilon = 1.0e-8);

}

#endif
