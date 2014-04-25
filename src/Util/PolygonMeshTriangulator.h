/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYGON_MESH_TRIANGULATOR_H_INCLUDED
#define CNOID_UTIL_POLYGON_MESH_TRIANGULATOR_H_INCLUDED

#include "SceneShape.h"
#include "exportdecl.h"

namespace cnoid {

class PolygonMeshTriangulatorImpl;

class CNOID_EXPORT PolygonMeshTriangulator
{
public:
    PolygonMeshTriangulator();
    PolygonMeshTriangulator(const PolygonMeshTriangulator& org);
    ~PolygonMeshTriangulator();

    void setDeepCopyEnabled(bool on);

    SgMeshPtr triangulate(SgPolygonMesh& polygonMesh);

    const std::string& errorMessage() const;

private:
    PolygonMeshTriangulatorImpl * impl;
};
}

#endif

