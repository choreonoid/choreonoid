/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MESH_NORMAL_GENERATOR_H
#define CNOID_UTIL_MESH_NORMAL_GENERATOR_H

#include "exportdecl.h"

namespace cnoid {

class SgMesh;
class MeshNormalGeneratorImpl;

class CNOID_EXPORT MeshNormalGenerator
{
public:
    MeshNormalGenerator();
    MeshNormalGenerator(const MeshNormalGenerator& org);
    ~MeshNormalGenerator();

    void setOverwritingEnabled(bool on);
    void setMinCreaseAngle(float angle);
    void setMaxCreaseAngle(float angle);

    bool generateNormals(SgMesh* mesh, float creaseAngle = 3.14159f, bool removeRedundantVertices = false);

private:
    MeshNormalGeneratorImpl* impl;
};

}

#endif
