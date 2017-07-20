/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshGenerator.h"
#include "MeshNormalGenerator.h"
#include "MeshExtractor.h"
#include "Triangulator.h"
#include "SceneDrawables.h"

using namespace std;
using namespace cnoid;

namespace {
const double PI = 3.14159265358979323846;
}

MeshGenerator::MeshGenerator()
{
    isNormalGenerationEnabled_ = true;
    normalGenerator = 0;
    divisionNumber_ = 20;
}


MeshGenerator::~MeshGenerator()
{
    if(normalGenerator){
        delete normalGenerator;
    }
}


void MeshGenerator::setDivisionNumber(int n)
{
    divisionNumber_ = n;
}


int MeshGenerator::divisionNumber() const
{
    return divisionNumber_;
}


void MeshGenerator::enableNormalGeneration(bool on)
{
    isNormalGenerationEnabled_ = on;
}


bool MeshGenerator::isNormalGenerationEnabled() const
{
    return isNormalGenerationEnabled_;
}


void MeshGenerator::generateNormals(SgMesh* mesh, double creaseAngle)
{
    if(isNormalGenerationEnabled_){
        if(!normalGenerator){
            normalGenerator = new MeshNormalGenerator;
        }
        normalGenerator->generateNormals(mesh, creaseAngle);
    }
}


SgMesh* MeshGenerator::generateBox(Vector3 size)
{
    if(size.x() < 0.0 || size.y() < 0.0 || size.z() < 0.0){
        return 0;
    }

    const float x = size.x() * 0.5;
    const float y = size.y() * 0.5;
    const float z = size.z() * 0.5;

    SgMesh* mesh = new SgMesh;
    
    SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve(8);

    vertices.push_back(Vector3f( x, y, z));
    vertices.push_back(Vector3f(-x, y, z));
    vertices.push_back(Vector3f(-x,-y, z));
    vertices.push_back(Vector3f( x,-y, z));
    vertices.push_back(Vector3f( x, y,-z));
    vertices.push_back(Vector3f(-x, y,-z));
    vertices.push_back(Vector3f(-x,-y,-z));
    vertices.push_back(Vector3f( x,-y,-z));

    mesh->reserveNumTriangles(12);
    mesh->addTriangle(0,1,2);
    mesh->addTriangle(2,3,0);
    mesh->addTriangle(0,5,1);
    mesh->addTriangle(0,4,5);
    mesh->addTriangle(1,5,6);
    mesh->addTriangle(1,6,2);
    mesh->addTriangle(2,6,7);
    mesh->addTriangle(2,7,3);
    mesh->addTriangle(3,7,4);
    mesh->addTriangle(3,4,0);
    mesh->addTriangle(4,6,5);
    mesh->addTriangle(4,7,6);

    mesh->setPrimitive(SgMesh::Box(size));
    mesh->updateBoundingBox();

    generateNormals(mesh, 0.0);

    return mesh;
}


SgMesh* MeshGenerator::generateSphere(double radius)
{
    if(radius < 0.0 || divisionNumber_ < 4){
        return 0;
    }

    SgMesh* mesh = new SgMesh();
    
    const int vdn = divisionNumber_ / 2;  // latitudinal division number
    const int hdn = divisionNumber_;      // longitudinal division number

    SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve((vdn - 1) * hdn + 2);

    for(int i=1; i < vdn; i++){ // latitudinal direction
        const double tv = i * PI / vdn;
        for(int j=0; j < hdn; j++){ // longitudinal direction
            const double th = j * 2.0 * PI / hdn;
            vertices.push_back(Vector3f(radius * sin(tv) * cos(th), radius * cos(tv), radius * sin(tv) * sin(th)));
        }
    }
    
    const int topIndex  = vertices.size();
    vertices.push_back(Vector3f(0.0f,  radius, 0.0f));
    const int bottomIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f, -radius, 0.0f));

    mesh->reserveNumTriangles(vdn * hdn * 2);

    // top faces
    for(int i=0; i < hdn; ++i){
        mesh->addTriangle(topIndex, (i+1) % hdn, i);
    }

    // side faces
    for(int i=0; i < vdn - 2; ++i){
        const int upper = i * hdn;
        const int lower = (i + 1) * hdn;
        for(int j=0; j < hdn; ++j) {
            // upward convex triangle
            mesh->addTriangle(j + upper, ((j + 1) % hdn) + lower, j + lower);
            // downward convex triangle
            mesh->addTriangle(j + upper, ((j + 1) % hdn) + upper, ((j + 1) % hdn) + lower);
        }
    }
    
    // bottom faces
    const int offset = (vdn - 2) * hdn;
    for(int i=0; i < hdn; ++i){
        mesh->addTriangle(bottomIndex, (i % hdn) + offset, ((i+1) % hdn) + offset);
    }

    mesh->setPrimitive(SgMesh::Sphere(radius));
    mesh->updateBoundingBox();

    //! \todo set normals directly without using the following function
    generateNormals(mesh, PI);

    return mesh;
}


SgMesh* MeshGenerator::generateCylinder(double radius, double height, bool bottom, bool top, bool side)
{
    if(height < 0.0 || radius < 0.0){
        return 0;
    }

    SgMesh* mesh = new SgMesh();
    
    SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.resize(divisionNumber_ * 2);

    const double y = height / 2.0;
    for(int i=0 ; i < divisionNumber_ ; i++ ){
        const double angle = i * 2.0 * PI / divisionNumber_;
        Vector3f& vtop = vertices[i];
        Vector3f& vbottom = vertices[i + divisionNumber_];
        vtop[0] = vbottom[0] = radius * cos(angle);
        vtop[2] = vbottom[2] = radius * sin(angle);
        vtop[1]    =  y;
        vbottom[1] = -y;
    }

    const int topCenterIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f,  y, 0.0f));
    const int bottomCenterIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f, -y, 0.0f));

    mesh->reserveNumTriangles(divisionNumber_ * 4);

    for(int i=0; i < divisionNumber_; ++i){
        // top face
        if(top){
            mesh->addTriangle(topCenterIndex, (i+1) % divisionNumber_, i);
        }
        // side face (upward convex triangle)
        if(side){        
            mesh->addTriangle(i, ((i+1) % divisionNumber_) + divisionNumber_, i + divisionNumber_);
            // side face (downward convex triangle)
            mesh->addTriangle(i, (i+1) % divisionNumber_, ((i + 1) % divisionNumber_) + divisionNumber_);
        }
        // bottom face
        if(bottom){
            mesh->addTriangle(bottomCenterIndex, i + divisionNumber_, ((i+1) % divisionNumber_) + divisionNumber_);
        }
    }

    mesh->setPrimitive(SgMesh::Cylinder(radius, height));
    mesh->updateBoundingBox();

    generateNormals(mesh, PI / 2.0);
    
    return mesh;
}


SgMesh* MeshGenerator::generateCone(double radius, double height, bool bottom, bool side)
{
    if(radius < 0.0 || height < 0.0){
        return 0;
    }

    SgMesh* mesh = new SgMesh();
    
    SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve(divisionNumber_ + 2);

    for(int i=0;  i < divisionNumber_; ++i){
        const double angle = i * 2.0 * PI / divisionNumber_;
        vertices.push_back(Vector3f(radius * cos(angle), -height / 2.0, radius * sin(angle)));
    }

    const int topIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f, height / 2.0, 0.0f));
    const int bottomCenterIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f, -height / 2.0, 0.0f));

    mesh->reserveNumTriangles(divisionNumber_ * 2);

    for(int i=0; i < divisionNumber_; ++i){
        // side faces
        if(side){
            mesh->addTriangle(topIndex, (i + 1) % divisionNumber_, i);
        }
        // bottom faces
        if(bottom){
            mesh->addTriangle(bottomCenterIndex, i, (i + 1) % divisionNumber_);
        }
    }

    mesh->setPrimitive(SgMesh::Cone(radius, height));
    mesh->updateBoundingBox();

    generateNormals(mesh, PI / 2.0);

    return mesh;
}


SgMesh* MeshGenerator::generateCapsule(double radius, double height)
{
    if(height < 0.0 || radius < 0.0){
        return 0;
    }

    SgMesh* mesh = new SgMesh();

    int vdn = divisionNumber_ / 2;  // latitudinal division number
    if(vdn%2)
        vdn +=1;

    const int hdn = divisionNumber_;      // longitudinal division number

    SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve( vdn * hdn + 2);

    for(int i=1; i < vdn+1; i++){ // latitudinal direction
        double y;
        double tv;
        if(i <= vdn / 2){
            y = height / 2.0;
            tv = i * PI / vdn;
        }else{
            y = - height / 2.0;
            tv = (i-1) * PI / vdn;
        }

        for(int j=0; j < hdn; j++){ // longitudinal direction
            const double th = j * 2.0 * PI / hdn;
            vertices.push_back(Vector3f(radius * sin(tv) * cos(th), radius * cos(tv) + y, radius * sin(tv) * sin(th)));
        }
    }

    const int topIndex  = vertices.size();
    vertices.push_back(Vector3f(0.0f,  radius + height /2.0, 0.0f));
    const int bottomIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f, -radius - height / 2.0, 0.0f));

    mesh->reserveNumTriangles(vdn * hdn * 2);

    // top faces
    for(int i=0; i < hdn; ++i){
        mesh->addTriangle(topIndex, (i+1) % hdn, i);
    }

    // side faces
    for(int i=0; i < vdn - 1; ++i){
        const int upper = i * hdn;
        const int lower = (i + 1) * hdn;
        for(int j=0; j < hdn; ++j) {
            // upward convex triangle
            mesh->addTriangle(j + upper, ((j + 1) % hdn) + lower, j + lower);
            // downward convex triangle
            mesh->addTriangle(j + upper, ((j + 1) % hdn) + upper, ((j + 1) % hdn) + lower);
        }
    }

    // bottom faces
    const int offset = (vdn - 1) * hdn;
    for(int i=0; i < hdn; ++i){
        mesh->addTriangle(bottomIndex, (i % hdn) + offset, ((i+1) % hdn) + offset);
    }

    mesh->setPrimitive(SgMesh::Capsule(radius, height));
    mesh->updateBoundingBox();

    generateNormals(mesh, PI / 2.0);

    return mesh;
}


SgMesh* MeshGenerator::generateDisc(double radius, double innerRadius)
{
    if(innerRadius <= 0.0 || radius <= innerRadius){
        return 0;
    }

    SgMesh* mesh = new SgMesh();

    SgVertexArray& vertices = *mesh->getOrCreateVertices();
    vertices.reserve(divisionNumber_ * 2);

    for(int i=0;  i < divisionNumber_; ++i){
        const double angle = i * 2.0 * PI / divisionNumber_;
        const double x = cos(angle);
        const double y = sin(angle);
        vertices.push_back(Vector3f(innerRadius * x, innerRadius * y, 0.0f));
        vertices.push_back(Vector3f(radius * x, radius * y, 0.0f));
    }

    mesh->reserveNumTriangles(divisionNumber_ * 2);
    mesh->getOrCreateNormals()->push_back(Vector3f::UnitZ());
    SgIndexArray& normalIndices = mesh->normalIndices();
    normalIndices.reserve(divisionNumber_ * 2 * 3);

    for(int i=0; i < divisionNumber_; ++i){
        const int j = (i + 1) % divisionNumber_;
        const int current = i * 2;
        const int next = j * 2;
        mesh->addTriangle(current, current + 1, next + 1);
        mesh->addTriangle(current, next + 1, next);
        for(int j=0; j < 6; ++j){
            normalIndices.push_back(0);
        }
    }

    mesh->updateBoundingBox();

    return mesh;
}


SgMesh* MeshGenerator::generateArrow(double cylinderRadius, double cylinderHeight, double coneRadius, double coneHeight)
{
    SgShapePtr cone = new SgShape;
    //setDivisionNumber(20);
    cone->setMesh(generateCone(coneRadius, coneHeight));
    SgPosTransform* conePos = new SgPosTransform;
    conePos->setTranslation(Vector3(0.0, cylinderHeight / 2.0 + coneHeight / 2.0, 0.0));
    conePos->addChild(cone);

    SgShapePtr cylinder = new SgShape;
    //setDivisionNumber(12);
    cylinder->setMesh(generateCylinder(cylinderRadius, cylinderHeight, true, false));
        
    MeshExtractor meshExtractor;
    SgGroupPtr group = new SgGroup;
    group->addChild(conePos);
    group->addChild(cylinder);
    return meshExtractor.integrate(group);
}


SgMesh* MeshGenerator::generateTorus(double radius, double crossSectionRadius)
{
    int divisionNumber2 = divisionNumber_ / 4;

    SgMesh* mesh = new SgMesh();
    SgVertexArray& vertices = *mesh->getOrCreateVertices();
    vertices.reserve(divisionNumber_ * divisionNumber2);

    for(int i=0; i < divisionNumber_; ++i){
        double phi = i * 2.0 * PI / divisionNumber_;
        for(int j=0; j < divisionNumber2; ++j){
            double theta = j * 2.0 * PI / divisionNumber2;
            Vector3f v;
            double r = crossSectionRadius * cos(theta) + radius;
            v.x() = cos(phi) * r;
            v.y() = crossSectionRadius * sin(theta);
            v.z() = sin(phi) * r;
            vertices.push_back(v);
        }
    }

    mesh->reserveNumTriangles(2 * divisionNumber_ * divisionNumber2);

    for(int i=0; i < divisionNumber_; ++i){
        int current = i * divisionNumber2;
        int next = ((i + 1) % divisionNumber_) * divisionNumber2;
        for(int j=0; j < divisionNumber2; ++j){
            int j_next = (j + 1) % divisionNumber2;
            mesh->addTriangle(current + j, next + j_next, next + j);
            mesh->addTriangle(current + j, current + j_next, next + j_next);
        }
    }

    mesh->updateBoundingBox();

    generateNormals(mesh, PI);

    return mesh;
}

SgMesh* MeshGenerator::generateExtrusion(const Extrusion& extrusion)
{
    int numSpines = extrusion.spine.size();
    int numCrosses = extrusion.crossSection.size();

    if(numSpines < 2 || numCrosses < 2){
        return 0;
    }
    
    bool isClosed = false;
    if(extrusion.spine[0][0] == extrusion.spine[numSpines - 1][0] &&
       extrusion.spine[0][1] == extrusion.spine[numSpines - 1][1] &&
       extrusion.spine[0][2] == extrusion.spine[numSpines - 1][2] ){
        isClosed = true;
        numSpines --;
    }
    bool isCrossSectionClosed = (extrusion.crossSection[0] == extrusion.crossSection[numCrosses - 1]);
    if(isCrossSectionClosed)
        numCrosses --;

    if(numSpines < 2 || numCrosses < 2){
        return 0;
    }

    SgMesh* mesh = new SgMesh;
    SgVertexArray& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve(numSpines * numCrosses);

    Vector3 preZaxis(Vector3::Zero());
    int definedZaxis = -1;
    vector<Vector3> Yaxisarray;
    vector<Vector3> Zaxisarray;
    if(numSpines > 2){
        for(int i=0; i < numSpines; ++i){
            Vector3 Yaxis, Zaxis;
            if(i == 0){
                if(isClosed){
                    const Vector3& spine1 = extrusion.spine[numSpines - 1];
                    const Vector3& spine2 = extrusion.spine[0];
                    const Vector3& spine3 = extrusion.spine[1];
                    Yaxis = spine3 - spine1;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                } else {
                    const Vector3& spine1 = extrusion.spine[0];
                    const Vector3& spine2 = extrusion.spine[1];
                    const Vector3& spine3 = extrusion.spine[2];
                    Yaxis = spine2 - spine1;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                }
            } else if(i == numSpines - 1){
                if(isClosed){
                    const Vector3& spine1 = extrusion.spine[numSpines - 2];
                    const Vector3& spine2 = extrusion.spine[numSpines - 1];
                    const Vector3& spine3 = extrusion.spine[0];
                    Yaxis = spine3 - spine1;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                } else {
                    const Vector3& spine1 = extrusion.spine[numSpines - 3];
                    const Vector3& spine2 = extrusion.spine[numSpines - 2];
                    const Vector3& spine3 = extrusion.spine[numSpines - 1];
                    Yaxis = spine3 - spine2;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                }
            } else {
                const Vector3& spine1 = extrusion.spine[i - 1];
                const Vector3& spine2 = extrusion.spine[i];
                const Vector3& spine3 = extrusion.spine[i + 1];
                Yaxis = spine3 - spine1;
                Zaxis = (spine3-spine2).cross(spine1-spine2);
            }
            if(!Zaxis.norm()){
                if(definedZaxis != -1)
                    Zaxis = preZaxis;
            } else {
                if(definedZaxis == -1){
                    definedZaxis = i;
                }
                preZaxis = Zaxis;
            }
            Yaxisarray.push_back(Yaxis);
            Zaxisarray.push_back(Zaxis);
        }
    } else {
        const Vector3 Yaxis(extrusion.spine[1] - extrusion.spine[0]);
        Yaxisarray.push_back(Yaxis);
        Yaxisarray.push_back(Yaxis);
    }

    const int numScales = extrusion.scale.size();
    const int numOrientations = extrusion.orientation.size();
    Vector3 scale(1.0, 0.0, 1.0);
    AngleAxis orientation(0.0, Vector3::UnitZ());

    for(int i=0; i < numSpines; ++i){
        Matrix3 Scp;
        Vector3 y = Yaxisarray[i].normalized();
        if(definedZaxis == -1){
            AngleAxis R(acos(y[1]), Vector3(y[2], 0.0, -y[0]));
            Scp = R.toRotationMatrix();
        } else {
            if(i < definedZaxis){
                Zaxisarray[i] = Zaxisarray[definedZaxis];
            }
            if(i && (Zaxisarray[i].dot(Zaxisarray[i - 1]) < 0.0)){
                Zaxisarray[i] *= -1.0;
            }
            Vector3 z = Zaxisarray[i].normalized();
            Vector3 x = y.cross(z);
            Scp << x, y, z;
        }

        const Vector3& spine = extrusion.spine[i];

        if(numScales == 1){
            scale << extrusion.scale[0][0], 0.0, extrusion.scale[0][1];
        } else if(numScales > 1){
            scale << extrusion.scale[i][0], 0.0, extrusion.scale[i][1];
        }
        if(numOrientations == 1){
            orientation = extrusion.orientation[0];
        } else if(numOrientations > 1){
            orientation = extrusion.orientation[i];
        }

        for(int j=0; j < numCrosses; ++j){
            const Vector3 crossSection(extrusion.crossSection[j][0], 0.0, extrusion.crossSection[j][1]);
            const Vector3 v1(crossSection[0] * scale[0], 0.0, crossSection[2] * scale[2]);
            const Vector3 v = Scp * orientation.toRotationMatrix() * v1 + spine;
            vertices.push_back(v.cast<float>());
        }
    }

    int numSpinePoints = numSpines;
    int numCrossPoints = numCrosses;

    if(isClosed)
        numSpinePoints++;
    if(isCrossSectionClosed)
        numCrossPoints++;

    for(int i=0; i < numSpinePoints - 1 ; ++i){
        int ii = (i + 1) % numSpines;
        int upper = i * numCrosses;
        int lower = ii * numCrosses;

        for(int j=0; j < numCrossPoints - 1; ++j) {
            int jj = (j + 1) % numCrosses;
            mesh->addTriangle(j + upper, j + lower, jj + lower);
            mesh->addTriangle(j + upper, jj + lower, jj + upper);
        }
    }

    Triangulator<SgVertexArray> triangulator;
    vector<int> polygon;
        
    if(extrusion.beginCap && !isClosed){
        triangulator.setVertices(vertices);
        polygon.clear();
        for(int i=0; i < numCrosses; ++i){
            polygon.push_back(i);
        }
        triangulator.apply(polygon);
        const vector<int>& triangles = triangulator.triangles();
        for(size_t i=0; i < triangles.size(); i += 3){
            mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+1]], polygon[triangles[i+2]]);
        }
    }

    if(extrusion.endCap && !isClosed){
        triangulator.setVertices(vertices);
        polygon.clear();
        for(int i=0; i < numCrosses; ++i){
            polygon.push_back(numCrosses * (numSpines - 1) + i);
        }
        triangulator.apply(polygon);
        const vector<int>& triangles = triangulator.triangles();
        for(size_t i=0; i < triangles.size(); i +=3){
            mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+2]], polygon[triangles[i+1]]);
        }
    }

    mesh->updateBoundingBox();

    generateNormals(mesh, extrusion.creaseAngle);

    return mesh;
}


SgLineSet* MeshGenerator::generateExtrusionLineSet(const Extrusion& extrusion, SgMesh* mesh)
{
    const int nc = extrusion.crossSection.size();
    const int ns = extrusion.spine.size();
    if(nc < 4 || ns < 2){
        return 0;
    }
    SgLineSet* lineSet = new SgLineSet;
    lineSet->setVertices(mesh->vertices());

    const int n = ns - 1;

    bool isCrossSectionClosed = (extrusion.crossSection[0] == extrusion.crossSection[nc - 1]);
    const int m = isCrossSectionClosed ? nc - 1 : nc;
    
    int o = 0;
    for(int i=0; i < n; ++i){
        for(int j=0; j < m; ++j){
            lineSet->addLine(o + j, o + (j + 1) % nc);
            lineSet->addLine(o + j, o + j + nc);
        }
        o += nc;
    }
    for(int j=0; j < m; ++j){
        lineSet->addLine(o + j, o + (j + 1) % nc);
    }

    return lineSet;
}


SgMesh* MeshGenerator::generateElevationGrid(const ElevationGrid& grid)
{
    SgMesh* mesh = new SgMesh;
    if(grid.xDimension * grid.zDimension != grid.height.size()){
        return mesh;
    }

    mesh->setVertices(new SgVertexArray());
    SgVertexArray& vertices = *mesh->vertices();
    vertices.reserve(grid.zDimension * grid.xDimension);

    for(int z=0; z < grid.zDimension; z++){
        for(int x=0; x < grid.xDimension; x++){
            vertices.push_back(Vector3f(x * grid.xSpacing, grid.height[z * grid.xDimension + x], z * grid.zSpacing));
        }
    }

    mesh->reserveNumTriangles((grid.zDimension - 1) * (grid.xDimension - 1) * 2);

    for(int z=0; z < grid.zDimension - 1; ++z){
        const int current = z * grid.xDimension;
        const int next = (z + 1) * grid.xDimension;
        for(int x=0; x < grid.xDimension - 1; ++x){
            if(grid.ccw){
                mesh->addTriangle( x + current, x + next, (x + 1) + next);
                mesh->addTriangle( x + current, (x + 1) + next, (x + 1) + current);
            }else{
                mesh->addTriangle( x + current, (x + 1) + next, x + next);
                mesh->addTriangle( x + current, (x + 1) + current, (x + 1) + next);
            }
        }
    }

    generateNormals(mesh, grid.creaseAngle);

    mesh->updateBoundingBox();

    return mesh;
}
