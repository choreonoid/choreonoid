/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshGenerator.h"
#include "MeshFilter.h"
#include "MeshExtractor.h"
#include "EigenUtil.h"
#include "Triangulator.h"

using namespace std;
using namespace cnoid;

namespace {

constexpr int defaultDivisionNumber = 20;

}

MeshGenerator::MeshGenerator()
{
    divisionNumber_ = ::defaultDivisionNumber;
    extraDivisionNumber_ = 1;
    extraDivisionMode_ = SgMesh::ExtraDivisionPreferred;
    isNormalGenerationEnabled_ = true;
    isBoundingBoxUpdateEnabled_ = true;
    meshFilter = nullptr;
}


MeshGenerator::~MeshGenerator()
{
    if(meshFilter){
        delete meshFilter;
    }
}


void MeshGenerator::setDivisionNumber(int n)
{
    if(n < 0){
        divisionNumber_ = ::defaultDivisionNumber;
    } else {
        divisionNumber_ = n;
    }
}


int MeshGenerator::defaultDivisionNumber()
{
    return ::defaultDivisionNumber;
}


void MeshGenerator::setNormalGenerationEnabled(bool on)
{
    isNormalGenerationEnabled_ = on;
}


void MeshGenerator::enableNormalGeneration(bool on)
{
    isNormalGenerationEnabled_ = on;
}


bool MeshGenerator::isNormalGenerationEnabled() const
{
    return isNormalGenerationEnabled_;
}


MeshFilter* MeshGenerator::getOrCreateMeshFilter()
{
    if(!meshFilter){
        meshFilter = new MeshFilter;
    }
    return meshFilter;
}


void MeshGenerator::generateNormals(SgMesh* mesh, double creaseAngle)
{
    if(isNormalGenerationEnabled_){
        getOrCreateMeshFilter()->generateNormals(mesh, creaseAngle);
    }
}


void MeshGenerator::setBoundingBoxUpdateEnabled(bool on)
{
    isBoundingBoxUpdateEnabled_ = on;
}


bool MeshGenerator::isBoundingBoxUpdateEnabled() const
{
    return isBoundingBoxUpdateEnabled_;
}


bool MeshGenerator::updateMeshWithPrimitiveInformation(SgMesh* mesh, int options)
{
    bool generated = false;
    switch(mesh->primitiveType()){
    case SgMesh::BoxType:
        generated = generateBox(mesh, options);
        break;
    case SgMesh::SphereType:
        generated = generateSphere(mesh, options);
        break;
    case SgMesh::CylinderType:
        generated = generateCylinder(mesh, options);
        break;
    case SgMesh::ConeType:
        generated = generateCone(mesh, options);
        break;
    case SgMesh::CapsuleType:
        generated = generateCapsule(mesh);
        break;
    default:
        break;
    }
    return generated;
}


SgMesh* MeshGenerator::generateBox(const Vector3& size, int options)
{
    SgMeshPtr mesh = new SgMesh;
    mesh->setPrimitive(SgMesh::Box(size));
    if(!generateBox(mesh, options)){
        mesh.reset();
    }
    return mesh.retn();
}


bool MeshGenerator::generateBox(SgMesh* mesh, int options)
{
    auto& box = mesh->primitive<SgMesh::Box>();
    auto& size = box.size;

    if(size.x() < 0.0 || size.y() < 0.0 || size.z() < 0.0){
        return false;
    }

    if(mesh->extraDivisionNumber() >= 2 || extraDivisionNumber_ >= 2){
        return generateBoxWithExtraTriangles(mesh);
    }

    const float x = size.x() * 0.5;
    const float y = size.y() * 0.5;
    const float z = size.z() * 0.5;

    mesh->setVertices(
        new SgVertexArray{
            {  x, y, z },
            { -x, y, z },
            { -x,-y, z },
            {  x,-y, z },
            {  x, y,-z },
            { -x, y,-z },
            { -x,-y,-z },
            {  x,-y,-z }
        });
        
    mesh->addTriangles({
            {0,1,2},{2,3,0},
            {0,5,1},{0,4,5},
            {1,5,6},{1,6,2},
            {2,6,7},{2,7,3},
            {3,7,4},{3,4,0},
            {4,6,5},{4,7,6}
        });
    
    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    generateNormals(mesh, 0.0);

    if(options & TextureCoordinate){
        generateTextureCoordinateForBox(mesh);
    }

    return true;
}


void MeshGenerator::generateTextureCoordinateForBox(SgMesh* mesh)
{
    mesh->setTexCoords(
        new SgTexCoordArray({
                { 0.0, 0.0 },
                { 1.0, 0.0 },
                { 0.0, 1.0 },
                { 1.0, 1.0 }
            }));

    mesh->texCoordIndices() = {
        3, 2, 0,
        0, 1, 3,
        1, 2, 0,
        1, 3, 2,
        3, 2, 0,
        3, 0, 1,
        2, 0, 1,
        2, 1, 3,
        0, 1, 3,
        0, 3, 2,
        2, 1, 3,
        2, 0, 1
    };
}


bool MeshGenerator::generateBoxWithExtraTriangles(SgMesh* mesh)
{
    const auto& box = mesh->primitive<SgMesh::Box>();
    const auto& size = box.size;

    int divisions[3];

    int edv;
    int mode;
    if(mesh->extraDivisionNumber() >= 2){
        edv = mesh->extraDivisionNumber();
        mode = mesh->extraDivisionMode();
    } else {
        edv = extraDivisionNumber_;
        mode = extraDivisionMode_;
    }
        
    if(mode == SgMesh::ExtraDivisionPreferred){
        double maxPatchSize = 0.0;
        for(int i=0; i < 3; ++i){
            double patchSize = size[i] / edv;
            if(patchSize > maxPatchSize){
                maxPatchSize = patchSize;
            }
        }
        for(int i=0; i < 3; ++i){
            divisions[i] = std::ceil(size[i] / maxPatchSize);
        }
    } else {
        for(int i=0; i < 3; ++i){
            if(mode & (1 << i)){
                divisions[i] = edv;
            } else {
                divisions[i] = 1;
            }
        }
    }

    auto vertices = mesh->getOrCreateVertices();

    vertices->reserve(
        2 * ((divisions[0] + 1) * (divisions[1] + 1) +
             (divisions[1] + 1) * (divisions[2] + 1) +
             (divisions[2] + 1) * (divisions[0] + 1)));
    
    mesh->reserveNumTriangles(
        4 * (divisions[0] * divisions[1] +
             divisions[1] * divisions[2] +
             divisions[2] * divisions[0]));

    // X-Y plane
    generateBoxPlaneMesh(size, mesh, vertices, 0, 1, 2, divisions);
    // Y-Z plane
    generateBoxPlaneMesh(size, mesh, vertices, 1, 2, 0, divisions);
    // X-Z plane
    generateBoxPlaneMesh(size, mesh, vertices, 2, 0, 1, divisions);

    /**!
       \note If the overlap vertices can be shared by the planes in the above process,
       the following function call can be removed to improve the performacne.
    */
    getOrCreateMeshFilter()->removeRedundantVertices(mesh);

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    generateNormals(mesh, 0.0);
    
    return true;
}


void MeshGenerator::generateBoxPlaneMesh
(const Vector3& size, SgMesh* mesh, SgVertexArray* vertices, int axis1, int axis2, int axis3, int* divisions)
{
    int n1 = divisions[axis1];
    int n2 = divisions[axis2];
    int m1 = n1 + 1;
    int m2 = n2 + 1;
    float size1 = size[axis1];
    float size2 = size[axis2];

    int topOffset = vertices->size();

    // generate vertices
    Vector3f v;
    for(int i=0; i < 2; ++i){
        v[axis3] = size[axis3] / 2.0;
        if(i == 1){
            v[axis3] = -v[axis3];
        }
        for(int j=0; j <= n2; ++j){
            v[axis2] = -(size2 / 2.0) + j * (size2 / n2);
            for(int k=0; k <= n1; ++k){
                v[axis1] = -(size1 / 2.0) + k * (size1 / n1);
                vertices->push_back(v);
            }
        }
    }

    // generate triangles
    for(int i=0; i < 2; ++i){
        int offset1 = topOffset;
        if(i == 1){
            offset1 += m1 * m2;
        }
        for(int j=0; j < n2; ++j){
            int offset = offset1 + j * m1;
            for(int k=0; k < n1; ++k){
                if(i == 0){
                    mesh->addTriangle(offset + k, offset + k + 1, offset + k + m1);
                    mesh->addTriangle(offset + k + 1, offset + k + 1 + m1, offset + k + m1);
                } else {
                    mesh->addTriangle(offset + k, offset + k + m1, offset + k + 1);
                    mesh->addTriangle(offset + k + 1, offset + k + m1, offset + k + 1 + m1);
                }
            }
        }
    }
}


SgMesh* MeshGenerator::generateSphere(double radius, int options)
{
    SgMeshPtr mesh = new SgMesh;
    mesh->setPrimitive(SgMesh::Sphere(radius));
    if(!generateSphere(mesh, options)){
        mesh.reset();
    }
    return mesh.retn();
}


bool MeshGenerator::generateSphere(SgMesh* mesh, int options)
{
    const auto& sphere = mesh->primitive<SgMesh::Sphere>();
    const auto radius = sphere.radius;
    
    if(radius <= 0.0){
        return false;
    }

    int dv;
    if(mesh->divisionNumber() >= 4){
        dv = mesh->divisionNumber();
    } else if(divisionNumber_ >= 4){
        dv = divisionNumber_;
    } else {
        dv = 4;
    }

    const int vdn = dv / 2;  // latitudinal division number
    const int hdn = dv;      // longitudinal division number

    auto& vertices = *mesh->setVertices(new SgVertexArray());
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

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    //! \todo set normals directly without using the following function
    generateNormals(mesh, PI);

    if(options & TextureCoordinate){
        generateTextureCoordinateForSphere(mesh);
    }

    return true;
}


/**
   \todo Check if the use of this inefficient function is rellay necessary.
*/
static int findTexCoordPoint(const SgTexCoordArray& texCoords, const Vector2f& point)
{
    for(size_t i=0; i < texCoords.size(); ++i){
        if(texCoords[i].isApprox(point)){
            return i;
        }
    }
    return -1;
}


void MeshGenerator::generateTextureCoordinateForSphere(SgMesh* mesh)
{
    const auto& sphere = mesh->primitive<SgMesh::Sphere>();
    const auto& vertices = *mesh->vertices();

    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();
    SgIndexArray& texCoordIndices = mesh->texCoordIndices();
    texCoordIndices.clear();

    Vector2f texPoint;
    int texIndex = 0;
    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        const Vector3f* point[3];
        bool over = false;
        double s[3] = { 0.0, 0.0, 0.0 };
        const auto triangle = mesh->triangle(i);
        for(int j=0; j < 3; ++j){
            point[j] = &vertices[triangle[j]];
            s[j] = ( atan2(point[j]->x(), point[j]->z()) + PI ) / 2.0 / PI;
            if(s[j] > 0.5){
                over = true;
            }
        }
        for(int j=0; j < 3; ++j){
            if(s[j] < 1.0e-6) s[j] = 0.0;
            if(s[j] > 1.0) s[j] = 1.0;
            if(over && s[j]==0.0){
                s[j] = 1.0;
            }
            double w = point[j]->y() / sphere.radius;
            if(w>1.0) w=1;
            if(w<-1.0) w=-1;
            texPoint << s[j], 1.0 - acos(w) / PI;

            int k = findTexCoordPoint(texCoords, texPoint);
            if(k >= 0){
                texCoordIndices.push_back(k);
            } else {
                texCoords.push_back(texPoint);
                texCoordIndices.push_back(texIndex++);
            }
        }
    }
}


SgMesh* MeshGenerator::generateCylinder(double radius, double height, int options)
{
    SgMeshPtr mesh = new SgMesh;
    mesh->setPrimitive(SgMesh::Cylinder(radius, height));
    if(!generateCylinder(mesh, options)){
        mesh.reset();
    }
    return mesh.retn();
}
    

bool MeshGenerator::generateCylinder(SgMesh* mesh, int options)
{
    const auto& cylinder = mesh->primitive<SgMesh::Cylinder>();
    
    if(cylinder.height <= 0.0 || cylinder.radius <= 0.0){
        return false;
    }

    auto& vertices = *mesh->getOrCreateVertices();
    int n;
    if(mesh->divisionNumber() >= 4){
        n = mesh->divisionNumber();
    } else if(divisionNumber_ >= 4){
        n = divisionNumber_;
    } else {
        n = 4;
    }
    int m = 1;
    if(cylinder.side){
        if(mesh->extraDivisionNumber() > 1){
            m = mesh->extraDivisionNumber();
        } else if(extraDivisionNumber_ > 1){
            m = extraDivisionNumber_;
        }
    }
    int size = n * (m + 1);
    int topCenterIndex;
    int bottomCenterIndex;
    if(cylinder.top){
        topCenterIndex = size;
        size += 1;
    }
    if(cylinder.bottom){
        bottomCenterIndex = size;
        size += 1;
    }
    vertices.resize(size);

    for(int i=0 ; i < n ; ++i){
        const double theta = i * 2.0 * PI / n;
        const double x = cylinder.radius * cos(theta);
        const double z = cylinder.radius * sin(theta);
        for(int j=0; j <= m; ++j){
            vertices[i + j * n] << x, (cylinder.height / 2.0 - j * (cylinder.height / m)), z;
        }
    }

    if(cylinder.top){
        vertices[topCenterIndex] << 0.0f, cylinder.height / 2.0, 0.0f;
    }
    if(cylinder.bottom){
        vertices[bottomCenterIndex] << 0.0f, -cylinder.height / 2.0, 0.0f;
    }

    int nt = 0;
    if(cylinder.top){
        nt += n;
    }
    if(cylinder.bottom){
        nt += n;
    }
    if(cylinder.side){
        nt += 2 * n * m;
    }
    mesh->reserveNumTriangles(nt);
    
    for(int i=0; i < n; ++i){
        // top face
        if(cylinder.top){
            mesh->addTriangle(topCenterIndex, (i + 1) % n, i);
        }
        // side face
        if(cylinder.side){
            for(int j=0; j < m; ++j){
                int offset = n * j;
                // upward convex triangle
                mesh->addTriangle(offset + i, offset + n + (i + 1) % n, offset + n + i);
                // side face (downward convex triangle)
                mesh->addTriangle(offset + i, offset + (i + 1) % n, offset + n + (i + 1) % n);
            }
        }
        // bottom face
        if(cylinder.bottom){
            mesh->addTriangle(bottomCenterIndex, m * n + i, m * n + (i + 1) % n);
        }
    }
    
    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    generateNormals(mesh, PI / 2.0);
    
    if((options & TextureCoordinate) && m == 1){
        generateTextureCoordinateForCylinder(mesh);
    }

    return true;
}


void MeshGenerator::generateTextureCoordinateForCylinder(SgMesh* mesh)
{
    const auto& vertices = *mesh->vertices();
    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();
    SgIndexArray& texCoordIndices = mesh->texCoordIndices();
    texCoordIndices.clear();

    Vector2f texPoint(0.5f, 0.5f); // center of top(bottom) index=0

    texCoords.push_back(texPoint);
    int texIndex = 1;
    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        Vector3f point[3];
        bool notside = true;
        int center = -1;
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        for(int j=0; j < 3; ++j){
            point[j] = vertices[triangle[j]];
            if(j > 0){
                if(point[0][1] != point[j][1]){
                    notside = false;
                }
            }
            if(point[j][0] == 0.0 && point[j][2] == 0.0){
                center = j;
            }
        }
        if(!notside){         //side
            bool over=false;
            Vector3f s(0.0, 0.0, 0.0);
            for(int j=0; j < 3; ++j){
                s[j] = ( atan2(point[j][0], point[j][2]) + PI ) / 2.0 / PI;
                if(s[j] > 0.5){
                    over = true;
                }
            }
            for(int j=0; j < 3; ++j){
                if(s[j] < 1.0e-6) s[j] = 0.0;
                if(s[j] > 1.0) s[j] = 1.0;
                if(over && s[j]==0.0){
                    s[j] = 1.0;
                }
                texPoint[0] = s[j];
                if(point[j][1] > 0.0){
                    texPoint[1] = 1.0;
                } else {
                    texPoint[1] = 0.0;
                }
                const int k = findTexCoordPoint(texCoords, texPoint);
                if(k >= 0){
                    texCoordIndices.push_back(k);
                }else{
                    texCoords.push_back(texPoint);
                    texCoordIndices.push_back(texIndex++);
                }
            }
        } else {              // top / bottom
            for(int j=0; j < 3; ++j){
                if(j!=center){
                    const double angle = atan2(point[j][2], point[j][0]);
                    texPoint[0] = 0.5 + 0.5 * cos(angle);
                    if(point[0][1] > 0.0){  //top
                        texPoint[1] = 0.5 - 0.5 * sin(angle);
                    } else {               //bottom
                        texPoint[1] = 0.5 + 0.5 * sin(angle);
                    }
                    const int k = findTexCoordPoint(texCoords, texPoint);
                    if(k != -1){
                        texCoordIndices.push_back(k);
                    }else{
                        texCoords.push_back(texPoint);
                        texCoordIndices.push_back(texIndex++);
                    }
                }else{
                    texCoordIndices.push_back(0);
                }
            }
        }
    }
}


SgMesh* MeshGenerator::generateCone(double radius, double height, int options)
{
    SgMeshPtr mesh = new SgMesh;
    mesh->setPrimitive(SgMesh::Cone(radius, height));
    if(!generateCone(mesh, options)){
        mesh.reset();
    }
    return mesh.retn();
}


bool MeshGenerator::generateCone(SgMesh* mesh, int options)
{
    const auto& cone = mesh->primitive<SgMesh::Cone>();
    
    if(cone.radius <= 0.0 || cone.height <= 0.0){
        return false;
    }

    int dv;
    if(mesh->divisionNumber() >= 4){
        dv = mesh->divisionNumber();
    } else if(divisionNumber_ >= 4){
        dv = divisionNumber_;
    } else {
        dv = 4;
    }

    auto& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve(dv + 2);

    for(int i=0;  i < dv; ++i){
        const double angle = i * 2.0 * PI / dv;
        vertices.push_back(Vector3f(cone.radius * cos(angle), -cone.height / 2.0, cone.radius * sin(angle)));
    }

    const int topIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f, cone.height / 2.0, 0.0f));
    const int bottomCenterIndex = vertices.size();
    vertices.push_back(Vector3f(0.0f, -cone.height / 2.0, 0.0f));

    mesh->reserveNumTriangles(dv * 2);

    for(int i=0; i < dv; ++i){
        // side faces
        if(cone.side){
            mesh->addTriangle(topIndex, (i + 1) % dv, i);
        }
        // bottom faces
        if(cone.bottom){
            mesh->addTriangle(bottomCenterIndex, i, (i + 1) % dv);
        }
    }

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    generateNormals(mesh, PI / 2.0);

    if(options & TextureCoordinate){
        generateTextureCoordinateForCone(mesh);
    }

    return true;
}


void MeshGenerator::generateTextureCoordinateForCone(SgMesh* mesh)
{
    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();
    SgIndexArray& texCoordIndices = mesh->texCoordIndices();
    texCoordIndices.clear();

    Vector2f texPoint(0.5, 0.5); //center of bottom index=0
    texCoords.push_back(texPoint);

    const auto& vertices = *mesh->vertices();

    int texIndex = 1;
    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        Vector3f point[3];
        int top = -1;
        int center = -1;
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        for(int j=0; j < 3; ++j){
            point[j] = vertices[triangle[j]];
            if(point[j][1] > 0.0){
                top = j;
            }
            if(point[j][0] == 0.0 && point[j][2] == 0.0){
                center = j;
            }
        }
        if(top >= 0){ //side
            Vector3f s(0.0f, 0.0f, 0.0f);
            int pre = -1;
            for(int j=0; j < 3; ++j){
                if(j != top){
                    s[j] = ( atan2(point[j][0], point[j][2]) + PI ) / 2.0 / PI;
                    if(pre != -1){
                        if(s[pre] > 0.5 && s[j] < 1.0e-6){
                            s[j] = 1.0;
                        }
                    }
                    pre = j;
                }
            }
            for(int j=0; j < 3; ++j){
                if(j != top){
                    texPoint << s[j], 0.0;
                } else {
                    texPoint << (s[0] + s[1] + s[2]) / 2.0, 1.0;
                }
                const int k = findTexCoordPoint(texCoords, texPoint);
                if(k != -1){
                    texCoordIndices.push_back(k);
                } else {
                    texCoords.push_back(texPoint);
                    texCoordIndices.push_back(texIndex++);
                }
            }
        } else { // bottom
            for(int j=0; j < 3; ++j){
                if(j != center){
                    const double angle = atan2(point[j][2], point[j][0]);
                    texPoint << 0.5 + 0.5 * cos(angle), 0.5 + 0.5 * sin(angle);
                    const int k = findTexCoordPoint(texCoords, texPoint);
                    if(k != -1){
                        texCoordIndices.push_back(k);
                    } else {
                        texCoords.push_back(texPoint);
                        texCoordIndices.push_back(texIndex++);
                    }
                } else {
                    texCoordIndices.push_back(0);
                }
            }
        }
    }
}


SgMesh* MeshGenerator::generateCapsule(double radius, double height)
{
    SgMeshPtr mesh = new SgMesh;
    mesh->setPrimitive(SgMesh::Capsule(radius, height));
    if(!generateCapsule(mesh)){
        mesh.reset();
    }
    return mesh.retn();
}
    

bool MeshGenerator::generateCapsule(SgMesh* mesh)
{
    const auto& capsule = mesh->primitive<SgMesh::Capsule>();
    const double height = capsule.height;
    const double radius = capsule.radius;
    
    if(height < 0.0 || radius <= 0.0){
        return false;
    }

    int dv;
    if(mesh->divisionNumber() >= 4){
        dv = mesh->divisionNumber();
    } else if(divisionNumber_ >= 4){
        dv = divisionNumber_;
    } else {
        dv = 4;
    }
    int edv = 1;
    if(mesh->extraDivisionNumber() > 1){
        edv = mesh->extraDivisionNumber();
    } else if(extraDivisionNumber_ > 1){
        edv = extraDivisionNumber_;
    }

    int vdn = dv / 2; // latitudinal division number
    if(vdn % 2){
        vdn +=1;
    }
    const int n = vdn + edv - 1;
    const int hdn = dv; // longitudinal division number
    int size = n * hdn + 2;
                
    auto& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve(size);

    // top half-sphere
    double y0 = height / 2.0;
    for(int i = 1; i <= vdn / 2; ++i){
        double theta  = i * PI / vdn;
        for(int j = 0; j < hdn; ++j){
            const double phi = j * 2.0 * PI / hdn;
            vertices.emplace_back(
                radius * sin(theta) * cos(phi),
                radius * cos(theta) + y0,
                radius * sin(theta) * sin(phi));
        }
    }
    // Cylinder pat divisions
    for(int i = 1; i < edv; ++i){
        for(int j = 0; j < hdn; ++j){
            const double phi = j * 2.0 * PI / hdn;
            vertices.emplace_back(
                radius * cos(phi),
                y0 - i * height / edv,
                radius * sin(phi));
        }
    }
    // bottom half-sphere
    y0 = -height / 2.0;
    for(int i = vdn / 2 + 1; i <= vdn; ++i){
        double theta  = (i - 1) * PI / vdn;
        for(int j = 0; j < hdn; ++j){
            const double phi = j * 2.0 * PI / hdn;
            vertices.emplace_back(
                radius * sin(theta) * cos(phi),
                radius * cos(theta) + y0,
                radius * sin(theta) * sin(phi));
        }
    }

    const int topIndex  = vertices.size();
    vertices.emplace_back(0.0f,  radius + height / 2.0, 0.0f);
    const int bottomIndex = vertices.size();
    vertices.emplace_back(0.0f, -radius - height / 2.0, 0.0f);

    mesh->reserveNumTriangles(n * hdn * 2);

    // top faces
    for(int i = 0; i < hdn; ++i){
        mesh->addTriangle(topIndex, (i + 1) % hdn, i);
    }
    // side faces
    for(int i = 0; i < n - 1; ++i){
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
    const int offset = (n - 1) * hdn;
    for(int i = 0; i < hdn; ++i){
        mesh->addTriangle(bottomIndex, (i % hdn) + offset, ((i + 1) % hdn) + offset);
    }

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    generateNormals(mesh, PI / 2.0);

    return mesh;
}


SgMesh* MeshGenerator::generateDisc(double radius, double innerRadius)
{
    if(innerRadius <= 0.0 || radius <= innerRadius){
        return nullptr;
    }

    auto mesh = new SgMesh;

    auto& vertices = *mesh->getOrCreateVertices();
    vertices.reserve(divisionNumber_ * 2);

    for(int i=0;  i < divisionNumber_; ++i){
        const double angle = i * 2.0 * PI / divisionNumber_;
        const double x = cos(angle);
        const double z = sin(angle);
        vertices.emplace_back(innerRadius * x, 0.0f, innerRadius * z);
        vertices.emplace_back(radius * x, 0.0f, radius * z);
    }

    mesh->reserveNumTriangles(divisionNumber_ * 2);
    mesh->getOrCreateNormals()->push_back(Vector3f::UnitZ());
    auto& normalIndices = mesh->normalIndices();
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

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    return mesh;
}


SgMesh* MeshGenerator::generateArrow(double cylinderRadius, double cylinderHeight, double coneRadius, double coneHeight)
{
    auto cone = new SgShape;
    //setDivisionNumber(20);
    generateCone(cone->setMesh(new SgMesh(SgMesh::Cone(coneRadius, coneHeight))), false);
    auto conePos = new SgPosTransform;
    conePos->setTranslation(Vector3(0.0, cylinderHeight / 2.0 + coneHeight / 2.0, 0.0));
    conePos->addChild(cone);

    auto cylinder = new SgShape;
    //setDivisionNumber(12);
    SgMesh::Cylinder cylinderParams(cylinderRadius, cylinderHeight);
    cylinderParams.top = false;
    generateCylinder(cylinder->setMesh(new SgMesh(cylinderParams)), false);
        
    MeshExtractor meshExtractor;
    SgGroupPtr group = new SgGroup;
    group->addChild(conePos);
    group->addChild(cylinder);

    auto arrow = meshExtractor.integrate(group);

    if(isBoundingBoxUpdateEnabled_){
        arrow->updateBoundingBox();
    }
    
    return arrow;
}


SgMesh* MeshGenerator::generateTorus(double radius, double crossSectionRadius)
{
    return generateTorus(radius, crossSectionRadius, 0.0, 2.0 * PI);
}


SgMesh* MeshGenerator::generateTorus(double radius, double crossSectionRadius, double beginAngle, double endAngle)
{
    bool isSemiTorus = (beginAngle > 0.0) || (endAngle < 2.0 * PI);
    int phiDivisionNumber = divisionNumber_ * endAngle / (2.0 * PI);
    double phiStep = (endAngle - beginAngle) / phiDivisionNumber;
    if(isSemiTorus){
        phiDivisionNumber += 1;
    }
    int thetaDivisionNumber = divisionNumber_ / 4;

    auto mesh = new SgMesh;
    auto& vertices = *mesh->getOrCreateVertices();
    vertices.reserve(phiDivisionNumber * thetaDivisionNumber);

    double phi = beginAngle;
    for(int i=0; i < phiDivisionNumber; ++i){
        for(int j=0; j < thetaDivisionNumber; ++j){
            double theta = j * 2.0 * PI / thetaDivisionNumber;
            Vector3f v;
            double r = crossSectionRadius * cos(theta) + radius;
            v.x() = cos(phi) * r;
            v.y() = crossSectionRadius * sin(theta);
            v.z() = sin(phi) * r;
            vertices.push_back(v);
        }
        phi += phiStep;
    }

    mesh->reserveNumTriangles(2 * phiDivisionNumber * thetaDivisionNumber);

    const int n = (endAngle == 2.0 * PI) ? phiDivisionNumber : (phiDivisionNumber - 1);
    for(int i=0; i < n; ++i){
        int current = i * thetaDivisionNumber;
        int next = ((i + 1) % phiDivisionNumber) * thetaDivisionNumber;
        for(int j=0; j < thetaDivisionNumber; ++j){
            int j_next = (j + 1) % thetaDivisionNumber;
            mesh->addTriangle(current + j, next + j_next, next + j);
            mesh->addTriangle(current + j, current + j_next, next + j_next);
        }
    }

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    generateNormals(mesh, PI);

    return mesh;
}


SgMesh* MeshGenerator::generateExtrusion(const Extrusion& extrusion, int meshOptions)
{
    int spineSize = extrusion.spine.size();
    int crossSectionSize = extrusion.crossSection.size();

    if(spineSize < 2 || crossSectionSize < 2){
        return nullptr;
    }
    
    bool isClosed = (extrusion.spine[0] == extrusion.spine[spineSize - 1]);
    if(isClosed)
        spineSize--;

    bool isCrossSectionClosed = (extrusion.crossSection[0] == extrusion.crossSection[crossSectionSize - 1]);
    if(isCrossSectionClosed)
        crossSectionSize --;

    if(spineSize < 2 || crossSectionSize < 2){
        return 0;
    }

    auto mesh = new SgMesh;
    auto& vertices = *mesh->setVertices(new SgVertexArray());
    vertices.reserve(spineSize * crossSectionSize);

    Vector3 preZaxis(Vector3::Zero());
    int definedZaxis = -1;
    vector<Vector3> Yaxisarray;
    vector<Vector3> Zaxisarray;
    if(spineSize > 2){
        for(int i=0; i < spineSize; ++i){
            Vector3 Yaxis, Zaxis;
            if(i == 0){
                if(isClosed){
                    const Vector3& spine1 = extrusion.spine[spineSize - 1];
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
            } else if(i == spineSize - 1){
                if(isClosed){
                    const Vector3& spine1 = extrusion.spine[spineSize - 2];
                    const Vector3& spine2 = extrusion.spine[spineSize - 1];
                    const Vector3& spine3 = extrusion.spine[0];
                    Yaxis = spine3 - spine1;
                    Zaxis = (spine3 - spine2).cross(spine1 - spine2);
                } else {
                    const Vector3& spine1 = extrusion.spine[spineSize - 3];
                    const Vector3& spine2 = extrusion.spine[spineSize - 2];
                    const Vector3& spine3 = extrusion.spine[spineSize - 1];
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

    for(int i=0; i < spineSize; ++i){
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

        for(int j=0; j < crossSectionSize; ++j){
            const Vector3 crossSection(extrusion.crossSection[j][0], 0.0, extrusion.crossSection[j][1]);
            const Vector3 v1(crossSection[0] * scale[0], 0.0, crossSection[2] * scale[2]);
            const Vector3 v = Scp * orientation.toRotationMatrix() * v1 + spine;
            vertices.push_back(v.cast<float>());
        }
    }

    int numSpinePoints = spineSize;
    int numCrossPoints = crossSectionSize;

    if(isClosed)
        numSpinePoints++;
    if(isCrossSectionClosed)
        numCrossPoints++;

    for(int i=0; i < numSpinePoints - 1 ; ++i){
        int ii = (i + 1) % spineSize;
        int upper = i * crossSectionSize;
        int lower = ii * crossSectionSize;

        for(int j=0; j < numCrossPoints - 1; ++j) {
            int jj = (j + 1) % crossSectionSize;
            mesh->addTriangle(j + upper, j + lower, jj + lower);
            mesh->addTriangle(j + upper, jj + lower, jj + upper);
        }
    }

    Triangulator<SgVertexArray> triangulator;
    vector<int> polygon;
        
    int numTriOfbeginCap = 0;
    int numTriOfendCap = 0;
    if(extrusion.beginCap && !isClosed){
        triangulator.setVertices(vertices);
        polygon.clear();
        for(int i=0; i < crossSectionSize; ++i){
            polygon.push_back(i);
        }
        triangulator.apply(polygon);
        const vector<int>& triangles = triangulator.triangles();
        numTriOfbeginCap = triangles.size() / 3;
        for(size_t i=0; i < triangles.size(); i += 3){
            mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+1]], polygon[triangles[i+2]]);
        }
    }

    if(extrusion.endCap && !isClosed){
        triangulator.setVertices(vertices);
        polygon.clear();
        for(int i=0; i < crossSectionSize; ++i){
            polygon.push_back(crossSectionSize * (spineSize - 1) + i);
        }
        triangulator.apply(polygon);
        const vector<int>& triangles = triangulator.triangles();
        numTriOfendCap = triangles.size() / 3;
        for(size_t i=0; i < triangles.size(); i +=3){
            mesh->addTriangle(polygon[triangles[i]], polygon[triangles[i+2]], polygon[triangles[i+1]]);
        }
    }

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    generateNormals(mesh, extrusion.creaseAngle);

    if(meshOptions & TextureCoordinate){
        generateTextureCoordinateForExtrusion(
            mesh, extrusion.crossSection, extrusion.spine,
            numTriOfbeginCap, numTriOfendCap, crossSectionSize*(spineSize-1));
    }

    return mesh;
}


void MeshGenerator::generateTextureCoordinateForExtrusion
(SgMesh* mesh, const Vector2Array& crossSection, const Vector3Array& spine,
 int numTriOfbeginCap, int numTriOfendCap, int indexOfendCap)
{
    const int spineSize = spine.size();
    const int crossSectionSize = crossSection.size();

    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();

    vector<double> s;
    vector<double> t;
    double slen = 0.0;
    s.push_back(0.0);
    for(int i=1; i < crossSectionSize; ++i){
        double x = crossSection[i][0] - crossSection[i-1][0];
        double z = crossSection[i][1] - crossSection[i-1][1];
        slen += sqrt(x*x + z*z);
        s.push_back(slen);
    }
    double tlen = 0.0;
    t.push_back(0.0);
    for(int i=1; i < spineSize; ++i){
        double x = spine[i][0] - spine[i-1][0];
        double y = spine[i][1] - spine[i-1][1];
        double z = spine[i][2] - spine[i-1][2];
        tlen += sqrt(x*x + y*y + z*z);
        t.push_back(tlen);
    }
    for(int i=0; i < spineSize; ++i){
        Vector2f point;
        point[1] = t[i] / tlen;
        for(int j=0; j < crossSectionSize; ++j){
            point[0] = s[j] / slen;
            texCoords.push_back(point);
        }
    }

    SgIndexArray& texCoordIndices = mesh->texCoordIndices();
    texCoordIndices.clear();
    for(int i=0; i < spineSize - 1 ; ++i){
        int ii = i + 1;
        int upper = i * crossSectionSize;
        int lower = ii * crossSectionSize;

        for(int j=0; j < crossSectionSize - 1; ++j) {
            int jj = j + 1;
            texCoordIndices.push_back(j + upper);
            texCoordIndices.push_back(j + lower);
            texCoordIndices.push_back(jj + lower);

            texCoordIndices.push_back(j + upper);
            texCoordIndices.push_back(jj + lower);
            texCoordIndices.push_back(jj + upper);
        }
    }

    if(!(numTriOfbeginCap+numTriOfendCap)){
        return;
    }
    const SgIndexArray& triangleVertices = mesh->triangleVertices();
    int endCapE = triangleVertices.size();
    int endCapS = endCapE - numTriOfendCap*3;
    int beginCapE = endCapS;
    int beginCapS = beginCapE - numTriOfbeginCap*3;

    double xmin, xmax;
    double zmin, zmax;
    xmin = xmax = crossSection[0][0];
    zmin = zmax = crossSection[0][1];
    for(size_t i=1; i < crossSection.size(); ++i){
        xmax = std::max(xmax, crossSection[i][0]);
        xmin = std::min(xmin, crossSection[i][0]);
        zmax = std::max(zmax, crossSection[i][1]);
        zmin = std::min(xmin, crossSection[i][1]);
    }
    float xsize = xmax - xmin;
    float zsize = zmax - zmin;

    if(numTriOfbeginCap){
        int endOfTexCoord = texCoords.size();
        for(int i=0; i < crossSectionSize; ++i){
            Vector2f point;
            point[0] = (crossSection[i][0] - xmin) / xsize;
            point[1] = (crossSection[i][1] - zmin) / zsize;
            texCoords.push_back(point);
        }
        for(int i = beginCapS; i <beginCapE ; ++i){
            texCoordIndices.push_back(triangleVertices[i] + endOfTexCoord);
        }
    }

    if(numTriOfendCap){
        int endOfTexCoord = texCoords.size();
        for(int i=0; i < crossSectionSize; ++i){
            Vector2f point;
            point[0] = (xmax - crossSection[i][0]) / xsize;
            point[1] = (crossSection[i][1] - zmin) / zsize;
            texCoords.push_back(point);
        }
        for(int i = endCapS; i < endCapE; ++i){
            texCoordIndices.push_back(triangleVertices[i]-indexOfendCap + endOfTexCoord);
        }
    }
}


SgLineSet* MeshGenerator::generateExtrusionLineSet(const Extrusion& extrusion, SgMesh* mesh)
{
    const int nc = extrusion.crossSection.size();
    const int ns = extrusion.spine.size();

    if(nc < 4 || ns < 2){
        return nullptr;
    }
    auto lineSet = new SgLineSet;
    lineSet->setVertices(mesh->vertices());

    const int n = ns - 1;

    bool isSpineClosed = (extrusion.spine[0] == extrusion.spine[ns - 1]);
    bool isCrossSectionClosed = (extrusion.crossSection[0] == extrusion.crossSection[nc - 1]);
    const int m = isCrossSectionClosed ? nc - 1 : nc;
    

    int o = 0;
    for(int i=0; i < n; ++i){
        for(int j=0; j < m; ++j){
            lineSet->addLine(o + j, o + (j + 1) % m);
            lineSet->addLine(o + j, o + j + m);
        }
        o += m;
    }
    if(!isSpineClosed){
        for(int j=0; j < m; ++j){
            lineSet->addLine(o + j, o + (j + 1) % m);
        }
    }

    return lineSet;
}


SgMesh* MeshGenerator::generateElevationGrid(const ElevationGrid& grid, int meshOptions)
{
    if(grid.xDimension * grid.zDimension != static_cast<int>(grid.height.size())){
        return nullptr;
    }

    auto mesh = new SgMesh;
    mesh->setVertices(new SgVertexArray());
    auto& vertices = *mesh->vertices();
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

    if(meshOptions & TextureCoordinate){
        generateTextureCoordinateForElevationGrid(mesh, grid);
    }

    if(isBoundingBoxUpdateEnabled_){
        mesh->updateBoundingBox();
    }

    return mesh;
}


void MeshGenerator::generateTextureCoordinateForElevationGrid(SgMesh* mesh, const ElevationGrid& grid)
{
    float xmax = grid.xSpacing * (grid.xDimension - 1);
    float zmax = grid.zSpacing * (grid.zDimension - 1);

    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();
    const SgVertexArray& vertices = *mesh->vertices();

    for(size_t i=0; i < vertices.size(); ++i){
        const Vector3f& v = vertices[i];
        texCoords.push_back(Vector2f(v.x() / xmax, v.z() / zmax));
    }
    mesh->texCoordIndices() = mesh->triangleVertices();
}


void MeshGenerator::generateTextureCoordinateForIndexedFaceSet(SgMeshBase* mesh)
{
    const SgVertexArray& vertices = *mesh->vertices();

    const Vector3f& v0 = vertices[0];
    Vector3f max = v0;
    Vector3f min = v0;

    const int n = vertices.size();
    for(int i=1; i < n; ++i){
        const Vector3f& vi = vertices[i];
        for(int j=0; j < 3; ++j){
            float vij = vi[j];
            if(vij > max[j]){
                max[j] = vij;
            } else if(vij < min[j]){
                min[j] = vij;
            }
        }
    }

    int s,t;
    const Vector3f size = max - min;
    if(size.x() >= size.y()){
        if(size.x() >= size.z()){
            s = 0;
            t = (size.y() >= size.z()) ? 1 : 2;
        } else {
            s = 2;
            t = 0;
        }
    } else {
        if(size.y() >= size.z()){
            s = 1;
            t = (size.x() >= size.z()) ? 0 : 2;
        } else {
            s = 2;
            t = 1;
        }
    }
    const float ratio = size[t] / size[s];

    mesh->setTexCoords(new SgTexCoordArray());
    SgTexCoordArray& texCoords = *mesh->texCoords();
    texCoords.resize(n);
    for(int i=0; i < n; ++i){
        texCoords[i] <<
            (vertices[i][s] - min[s]) / size[s],
            (vertices[i][t] - min[t]) / size[t] * ratio;
    }

    // Is this really necessary for rendering?
    mesh->texCoordIndices() = mesh->faceVertexIndices();
}
