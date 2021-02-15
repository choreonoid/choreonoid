/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MESH_GENERATOR_H
#define CNOID_UTIL_MESH_GENERATOR_H

#include "EigenTypes.h"
#include "SceneDrawables.h"
#include "exportdecl.h"

namespace cnoid {

class MeshFilter;

class CNOID_EXPORT MeshGenerator
{
public:
    MeshGenerator();
    MeshGenerator(const MeshGenerator& org);
    ~MeshGenerator();

    void setDivisionNumber(int n);
    int divisionNumber() const { return divisionNumber_; }
    static int defaultDivisionNumber();

    // Deprecated. Use SgMesh::setExtraDivisionNumber and SgMesh::setExtraDivisionMode
    //! \param mode A combination of SgMesh::ExtraDivisionMode symbols
    void setExtraDivisionNumber(int n, int mode = SgMesh::ExtraDivisionPreferred){
        extraDivisionNumber_ = n;
        extraDivisionMode_ = mode;
    }
    int extraDivisionNumber() const { return extraDivisionNumber_; }
    int extraDivisionMode() const { return extraDivisionMode_; }

    void setNormalGenerationEnabled(bool on);
    [[deprecated("Use setNormalGenerationEnabled")]]
    void enableNormalGeneration(bool on);
    bool isNormalGenerationEnabled() const;

    void setBoundingBoxUpdateEnabled(bool on);
    bool isBoundingBoxUpdateEnabled() const;

    enum MeshOption {
        NoOption = 0,
        TextureCoordinate = 1
    };

    bool updateMeshWithPrimitiveInformation(SgMesh* mesh, int options = NoOption);

    SgMesh* generateBox(const Vector3& size, int options = NoOption);
    SgMesh* generateSphere(double radius, int options = NoOption);
    SgMesh* generateCylinder(double radius, double height, int options = NoOption);
    SgMesh* generateCone(double radius, double height, int options = NoOption);
    SgMesh* generateCapsule(double radius, double height);
    SgMesh* generateDisc(double radius, double innerRadius);
    SgMesh* generateArrow(double cylinderRadius, double cylinderHeight, double coneRadius, double coneHeight);
    SgMesh* generateTorus(double radius, double crossSectionRadius);
    SgMesh* generateTorus(double radius, double crossSectionRadius, double beginAngle, double endAngle);

    typedef std::vector<Vector2, Eigen::aligned_allocator<Vector2>> Vector2Array;
    typedef std::vector<Vector3, Eigen::aligned_allocator<Vector3>> Vector3Array;
    typedef std::vector<AngleAxis, Eigen::aligned_allocator<AngleAxis>> AngleAxisArray;

    struct Extrusion
    {
        Vector2Array crossSection;
        Vector3Array spine;
        AngleAxisArray orientation;
        Vector2Array scale;
        double creaseAngle;
        bool beginCap;
        bool endCap;
        Extrusion() {
            creaseAngle = 0.0;
            beginCap = true;
            endCap = true;
        }
    };

    SgMesh* generateExtrusion(const Extrusion& extrusion, int meshOptions = NoOption);
    SgLineSet* generateExtrusionLineSet(const Extrusion& extrusion, SgMesh* mesh);

    struct ElevationGrid
    {
        int xDimension;
        int zDimension;
        double xSpacing;
        double zSpacing;
        std::vector<double> height;
        bool ccw;
        double creaseAngle;
        ElevationGrid() {
            xDimension = 0;
            zDimension = 0;
            xSpacing = 1.0;
            zSpacing = 1.0;
            ccw = true;
            creaseAngle = 0.0;
        }
    };

    SgMesh* generateElevationGrid(const ElevationGrid& elevationGrid, int meshOptions = NoOption);

    void generateTextureCoordinateForIndexedFaceSet(SgMeshBase* mesh);

private:
    int divisionNumber_;
    int extraDivisionNumber_;
    int extraDivisionMode_;
    bool isNormalGenerationEnabled_;
    bool isBoundingBoxUpdateEnabled_;
    MeshFilter* meshFilter;

    MeshFilter* getOrCreateMeshFilter();
    void generateNormals(SgMesh* mesh, double creaseAngle);
    bool generateBox(SgMesh* mesh, int options);
    bool generateBoxWithExtraTriangles(SgMesh* mesh);
    void generateTextureCoordinateForBox(SgMesh* mesh);
    void generateBoxPlaneMesh(
        const Vector3& size, SgMesh* mesh, SgVertexArray* vertices, int axis1, int axis2, int axis3, int* divisions);
    bool generateSphere(SgMesh* mesh, int options);
    bool generateCylinder(SgMesh* mesh, int options);
    bool generateCone(SgMesh* mesh, int options);
    bool generateCapsule(SgMesh* mesh);
    void generateTextureCoordinateForSphere(SgMesh* mesh);
    void generateTextureCoordinateForCylinder(SgMesh* mesh);
    void generateTextureCoordinateForCone(SgMesh* mesh);
    void generateTextureCoordinateForExtrusion(
        SgMesh* mesh, const Vector2Array& crossSections, const Vector3Array& spinePoints,
        int numTriOfbeginCap, int numTriOfendCap, int indexOfendCap);
    void generateTextureCoordinateForElevationGrid(SgMesh* mesh, const ElevationGrid& grid);
};

}

#endif
