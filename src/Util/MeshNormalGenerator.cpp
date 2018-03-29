/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshNormalGenerator.h"
#include "SceneDrawables.h"

using namespace std;
using namespace cnoid;

namespace {
const float PI = 3.14159265358979323846f;
}

namespace cnoid {

class MeshNormalGeneratorImpl
{
public:
    float minCreaseAngle;
    float maxCreaseAngle;
    SgNormalArrayPtr faceNormals;
    vector< vector<int> > vertexIndexToTriangleIndicesMap;
    vector< vector<int> > vertexIndexToNormalIndicesMap;
    bool isOverwritingEnabled;

    MeshNormalGeneratorImpl();
    MeshNormalGeneratorImpl(const MeshNormalGeneratorImpl& org);
    void removeRedundantVertices(SgMesh* mesh);
    void calculateFaceNormals(SgMesh* mesh);
    void setVertexNormals(SgMesh* mesh, float creaseAngle);
};
}


MeshNormalGenerator::MeshNormalGenerator()
{
    impl = new MeshNormalGeneratorImpl();
}


MeshNormalGeneratorImpl::MeshNormalGeneratorImpl()
{
    isOverwritingEnabled = false;
    minCreaseAngle = 0.0f;
    maxCreaseAngle = PI;
}


MeshNormalGenerator::MeshNormalGenerator(const MeshNormalGenerator& org)
{
    impl = new MeshNormalGeneratorImpl(*org.impl);
}


MeshNormalGeneratorImpl::MeshNormalGeneratorImpl(const MeshNormalGeneratorImpl& org)
{
    isOverwritingEnabled = org.isOverwritingEnabled;
    minCreaseAngle = org.minCreaseAngle;
    maxCreaseAngle = org.maxCreaseAngle;
}


MeshNormalGenerator::~MeshNormalGenerator()
{
    delete impl;
}


void MeshNormalGenerator::setOverwritingEnabled(bool on)
{
    impl->isOverwritingEnabled = on;
}


void MeshNormalGenerator::setMinCreaseAngle(float angle)
{
    impl->minCreaseAngle = angle;
}


void MeshNormalGenerator::setMaxCreaseAngle(float angle)
{
    impl->maxCreaseAngle = angle;
}


bool MeshNormalGenerator::generateNormals(SgMesh* mesh, float creaseAngle, bool removeRedundantVertices)
{
    if(!mesh->vertices() || mesh->triangleVertices().empty()){
        return false;
    }
    if(!impl->isOverwritingEnabled && mesh->normals() && !mesh->normals()->empty()){
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


void MeshNormalGeneratorImpl::removeRedundantVertices(SgMesh* mesh)
{
    const SgVertexArray& orgVertices = *mesh->vertices();
    const int numVertices = orgVertices.size();
    SgVertexArrayPtr newVerticesptr = new SgVertexArray();
    SgVertexArray& newVertices = *newVerticesptr;
    vector<int> indexMap(numVertices);

    for(int i=0; i<numVertices; i++){
        bool found = false;
        for(int j=0; j<newVertices.size(); j++){
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

    if(newVertices.size()==numVertices)
        return;

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
    for(int i=0; i < numTriangles; i++){
        SgMesh::TriangleRef triangle = mesh->triangle(i);
        for(int j=0; j<3; j++){
            triangle[j] = indexMap[triangle[j]];
        }
    }

}


void MeshNormalGeneratorImpl::calculateFaceNormals(SgMesh* mesh)
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
    

void MeshNormalGeneratorImpl::setVertexNormals(SgMesh* mesh, float givenCreaseAngle)
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
