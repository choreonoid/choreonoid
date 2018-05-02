/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshFilter.h"
#include "MeshExtractor.h"
#include "SceneDrawables.h"
#include <unordered_set>
#include <functional>

using namespace std;
using namespace cnoid;

namespace {

const float PI = 3.14159265358979323846f;

}

namespace std {

template<> struct hash<Array3i>
{
    void hash_combine(std::size_t& seed, int v) const {
        std::hash<int> hasher;
        seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
    }
    std::size_t operator()(const Array3i& value) const {
        std::size_t seed = 0;
        hash_combine(seed, value[0]);
        hash_combine(seed, value[1]);
        hash_combine(seed, value[2]);
        return seed;
    }
};

}


namespace cnoid {

class MeshFilterImpl
{
public:
    float minCreaseAngle;
    float maxCreaseAngle;
    SgNormalArrayPtr faceNormals;
    vector<vector<int>> vertexIndexToTriangleIndicesMap;
    vector<vector<int>> vertexIndexToNormalIndicesMap;
    unique_ptr<MeshExtractor> meshExtractor;
    bool isNormalOverwritingEnabled;

    MeshFilterImpl();
    MeshFilterImpl(const MeshFilterImpl& org);
    void forAllMeshes(SgNode* node, function<void(SgMesh* mesh)> callback);
    void removeRedundantVertices(SgMesh* mesh);
    void removeRedundantFaces(SgMesh* mesh);
    void calculateFaceNormals(SgMesh* mesh);
    void setVertexNormals(SgMesh* mesh, float creaseAngle);
};

}


MeshFilter::MeshFilter()
{
    impl = new MeshFilterImpl();
}


MeshFilterImpl::MeshFilterImpl()
{
    isNormalOverwritingEnabled = false;
    minCreaseAngle = 0.0f;
    maxCreaseAngle = PI;
}


MeshFilter::MeshFilter(const MeshFilter& org)
{
    impl = new MeshFilterImpl(*org.impl);
}


MeshFilterImpl::MeshFilterImpl(const MeshFilterImpl& org)
{
    isNormalOverwritingEnabled = org.isNormalOverwritingEnabled;
    minCreaseAngle = org.minCreaseAngle;
    maxCreaseAngle = org.maxCreaseAngle;
}


MeshFilter::~MeshFilter()
{
    delete impl;
}


void MeshFilter::setNormalOverwritingEnabled(bool on)
{
    impl->isNormalOverwritingEnabled = on;
}


void MeshFilter::setOverwritingEnabled(bool on)
{
    setNormalOverwritingEnabled(on);
}


void MeshFilter::setMinCreaseAngle(float angle)
{
    impl->minCreaseAngle = angle;
}


void MeshFilter::setMaxCreaseAngle(float angle)
{
    impl->maxCreaseAngle = angle;
}


void MeshFilterImpl::forAllMeshes(SgNode* node, function<void(SgMesh* mesh)> callback)
{
    if(!meshExtractor){
        meshExtractor.reset(new MeshExtractor);
    }
    meshExtractor->extract(node, callback);
}


bool MeshFilter::generateNormals(SgMesh* mesh, float creaseAngle, bool removeRedundantVertices)
{
    if(!mesh->vertices() || mesh->triangleVertices().empty()){
        return false;
    }
    if(!impl->isNormalOverwritingEnabled && mesh->normals() && !mesh->normals()->empty()){
        return false;
    }

    if(!impl->faceNormals){
        impl->faceNormals = new SgNormalArray;
    }
    if(removeRedundantVertices){
        impl->removeRedundantVertices(mesh);
    }
    impl->calculateFaceNormals(mesh);
    impl->setVertexNormals(mesh, creaseAngle);

    return true;
}


void MeshFilter::removeRedundantVertices(SgNode* scene)
{
    impl->forAllMeshes(scene, [&](SgMesh* mesh){ impl->removeRedundantVertices(mesh); });
}


void MeshFilter::removeRedundantVertices(SgMesh* mesh)
{
    impl->removeRedundantVertices(mesh);
}


void MeshFilterImpl::removeRedundantVertices(SgMesh* mesh)
{
    const SgVertexArray& orgVertices = *mesh->vertices();
    const int numVertices = orgVertices.size();
    SgVertexArrayPtr newVerticesptr = new SgVertexArray();
    SgVertexArray& newVertices = *newVerticesptr;
    vector<int> indexMap(numVertices);

    for(int i=0; i< numVertices; ++i){
        bool found = false;
        for(int j=0; j < newVertices.size(); ++j){
            if(orgVertices[i].isApprox(newVertices[j])){
                indexMap[i] = j;
                found = true;
                break;
            }
        }
        if(!found){
            indexMap[i] = newVertices.size();
            newVertices.push_back(orgVertices[i]);
        }
    }

    if(newVertices.size() == numVertices){
        return;
    }

    mesh->setVertices(newVerticesptr);

    if(mesh->hasNormals()){
        if(mesh->normalIndices().empty()){
            mesh->normalIndices() = mesh->triangleVertices();
        }
    }

    if(mesh->hasTexCoords()){
        if(mesh->texCoordIndices().empty()){
            mesh->texCoordIndices() = mesh->triangleVertices();
        }
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        for(int j=0; j < 3; ++j){
            triangle[j] = indexMap[triangle[j]];
        }
    }
}


void MeshFilter::removeRedundantFaces(SgNode* scene)
{
    impl->forAllMeshes(scene, [&](SgMesh* mesh){ impl->removeRedundantFaces(mesh); });
}


void MeshFilter::removeRedundantFaces(SgMesh* mesh)
{
    impl->removeRedundantFaces(mesh);
}


void MeshFilterImpl::removeRedundantFaces(SgMesh* mesh)
{
    unordered_set<Array3i> existingFaces;

    const auto orgTriangles = mesh->triangleVertices();
    auto& triangles = mesh->triangleVertices();
    triangles.clear();
    for(size_t i=0; i < orgTriangles.size(); ++i){
        Array3i triangle(orgTriangles[i * 3]);

        // sort triangle elements

        /*
        auto result = existingFaces.insert(triangle);
        if(result.second){
            mesh->newTriangle() = triangle;
        }
        */
    }
}


void MeshFilterImpl::calculateFaceNormals(SgMesh* mesh)
{
    const SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    const int numTriangles = mesh->numTriangles();

    faceNormals->clear();
    faceNormals->reserve(numTriangles);
    vertexIndexToTriangleIndicesMap.clear();
    vertexIndexToTriangleIndicesMap.resize(numVertices);

    for(int i=0; i < numTriangles; ++i){

        SgMesh::TriangleRef triangle = mesh->triangle(i);
        
        const Vector3f& v0 = vertices[triangle[0]];
        const Vector3f& v1 = vertices[triangle[1]];
        const Vector3f& v2 = vertices[triangle[2]];
        Vector3f normal((v1 - v0).cross(v2 - v0));
        // prevent NaN
        if (normal.norm() == 0){
          normal = Vector3f::UnitZ(); // Is this OK?
        }else{
          normal.normalize();
        }
        faceNormals->push_back(normal);

        for(int j=0; j < 3; ++j){
            vector<int>& trianglesOfVertex = vertexIndexToTriangleIndicesMap[triangle[j]];

            /**
               \todo Angle between adjacent edges should be taken into account
               to generate natural normals
            */
            bool isSameNormalFaceFound = false;
            for(size_t k=0; k < trianglesOfVertex.size(); ++k){
                const Vector3f& otherNormal = (*faceNormals)[trianglesOfVertex[k]];
                // the same face is not appended
                if(otherNormal.isApprox(normal, 5.0e-4)){
                    isSameNormalFaceFound = true;
                    break;
                }
            }
            if(!isSameNormalFaceFound){
                trianglesOfVertex.push_back(i);
            }
        }
    }
}
    

void MeshFilterImpl::setVertexNormals(SgMesh* mesh, float givenCreaseAngle)
{
    const float creaseAngle = std::max(minCreaseAngle, std::min(maxCreaseAngle, givenCreaseAngle));
    
    const SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    const int numTriangles = mesh->numTriangles();

    mesh->setNormals(new SgNormalArray());
    SgNormalArray& normals = *mesh->normals();
    SgIndexArray& normalIndices = mesh->normalIndices();
    normalIndices.clear();
    normalIndices.reserve(mesh->triangleVertices().size());

    vertexIndexToNormalIndicesMap.clear();
    vertexIndexToNormalIndicesMap.resize(numVertices);

    for(int faceIndex=0; faceIndex < numTriangles; ++faceIndex){

        SgMesh::TriangleRef triangle = mesh->triangle(faceIndex);

        for(int i=0; i < 3; ++i){

            const int vertexIndex = triangle[i];
            const vector<int>& trianglesOfVertex = vertexIndexToTriangleIndicesMap[vertexIndex];
            const Vector3f& currentFaceNormal = (*faceNormals)[faceIndex];
            Vector3f normal = currentFaceNormal;
            bool normalIsFaceNormal = true;
                
            // avarage normals of the faces whose crease angle is below the 'creaseAngle' variable
            for(size_t j=0; j < trianglesOfVertex.size(); ++j){
                const int adjacentFaceIndex = trianglesOfVertex[j];
                const Vector3f& adjacentFaceNormal = (*faceNormals)[adjacentFaceIndex];
                float cosAngle = currentFaceNormal.dot(adjacentFaceNormal)
                  / (currentFaceNormal.norm() * adjacentFaceNormal.norm());
                //prevent NaN
                if (cosAngle >  1.0) cosAngle =  1.0;
                if (cosAngle < -1.0) cosAngle = -1.0;
                const float angle = acosf(cosAngle);
                if(angle > 0.0f && angle < creaseAngle){
                    normal += adjacentFaceNormal;
                    normalIsFaceNormal = false;
                }
            }
            if(!normalIsFaceNormal){
                normal.normalize();
            }
            
            int normalIndex = -1;
            
            for(int j=0; j < 3; ++j){
                const int vertexIndex2 = triangle[j];
                const vector<int>& normalIndicesOfVertex = vertexIndexToNormalIndicesMap[vertexIndex2];
                for(size_t k=0; k < normalIndicesOfVertex.size(); ++k){
                    int index = normalIndicesOfVertex[k];
                    if(normals[index].isApprox(normal)){
                        normalIndex = index;
                        goto normalIndexFound;
                    }
                }
            }
            if(normalIndex < 0){
                normalIndex = normals.size();
                normals.push_back(normal);
                vertexIndexToNormalIndicesMap[vertexIndex].push_back(normalIndex);
            }
            
    normalIndexFound:
            normalIndices.push_back(normalIndex);
        }
    }
}
