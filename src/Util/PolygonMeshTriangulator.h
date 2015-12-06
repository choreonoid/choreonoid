/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_POLYGON_MESH_TRIANGULATOR_H
#define CNOID_UTIL_POLYGON_MESH_TRIANGULATOR_H

#include <string>
#include "exportdecl.h"

namespace cnoid {

class SgMesh;
class SgPolygonMesh;
class PolygonMeshTriangulatorImpl;

class CNOID_EXPORT PolygonMeshTriangulator
{
public:
    PolygonMeshTriangulator();
    PolygonMeshTriangulator(const PolygonMeshTriangulator& org);
    ~PolygonMeshTriangulator();

    void setDeepCopyEnabled(bool on);

    SgMesh* triangulate(SgPolygonMesh* polygonMesh);

    const std::string& errorMessage() const;

private:
    PolygonMeshTriangulatorImpl * impl;
};

}

#endif

