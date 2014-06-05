/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MESH_GENERATOR_H
#define CNOID_UTIL_MESH_GENERATOR_H

#include "SceneShape.h"
#include <Eigen/StdVector>
#include "exportdecl.h"

namespace cnoid {

class MeshNormalGenerator;

class CNOID_EXPORT MeshGenerator
{
public:
    MeshGenerator();
    MeshGenerator(const MeshGenerator& org);
    ~MeshGenerator();

    void setDivisionNumber(int n);
    int divisionNumber() const;

    void enableNormalGeneration(bool on);
    bool isNormanGenerationEnabled() const;

    SgMesh* generateBox(Vector3 size);
    SgMesh* generateSphere(double radius);
    SgMesh* generateCylinder(double radius, double height, bool bottom = true, bool top = true, bool side = true);
    SgMesh* generateCone(double radius, double height, bool bottom = true, bool side = true);
    SgMesh* generateDisc(double radius, double innerRadius);

    typedef std::vector<Vector2, Eigen::aligned_allocator<Vector2> > Vector2Array;
    typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3> > Vector3Array;
    typedef std::vector<AngleAxis, Eigen::aligned_allocator<AngleAxis> > AngleAxisArray;
        
    struct Extrusion
    {
        Vector2Array crossSection;
        Vector2Array scale;
        Vector3Array spine;
        AngleAxisArray orientation;
        double creaseAngle;
        bool beginCap;
        bool endCap;
        Extrusion() {
            creaseAngle = 0.0;
            beginCap = true;
            endCap = true;
        }
    };
            
    SgMesh* generateExtrusion(const Extrusion& extrusion);
    SgLineSet* generateExtrusionLineSet(const Extrusion& extrusion, SgMesh* mesh);

private:
    int divisionNumber_;
    bool isNormanGenerationEnabled_;
    MeshNormalGenerator* normalGenerator;

    void generateNormals(SgMesh* mesh, double creaseAngle);
};
}

#endif

                              

