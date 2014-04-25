/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MESH_NORMAL_GENERATOR_H_INCLUDED
#define CNOID_UTIL_MESH_NORMAL_GENERATOR_H_INCLUDED

#include "SceneShape.h"
#include "exportdecl.h"

namespace cnoid {

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

    bool generateNormals(SgMesh* mesh, float creaseAngle = 3.14159f);

private:
    MeshNormalGeneratorImpl* impl;
};
}

#endif
