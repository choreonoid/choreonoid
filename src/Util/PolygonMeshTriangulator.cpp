/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "PolygonMeshTriangulator.h"
#include "Triangulator.h"
#include "SceneDrawables.h"
#include <fmt/format.h>

using namespace std;
using namespace cnoid;

namespace cnoid {

class PolygonMeshTriangulatorImpl
{
public:
    bool isDeepCopyEnabled;
    Triangulator<SgVertexArray> triangulator;
    std::vector<int> polygon;
    std::vector<int> newIndexPositionToOrgPositionWithDelimitersMap;
    std::vector<int> newIndexPositionToOrgPositionMap;
    std::string errorMessage;

    void addErrorMessage(const std::string& message){
        if(!errorMessage.empty()){
            errorMessage += "\n";
        }
        errorMessage += message;
    }
    void addErrorMessage(const char* message){
        if(!errorMessage.empty()){
            errorMessage += "\n";
        }
        errorMessage += message;
    }

    PolygonMeshTriangulatorImpl();
    SgMesh* triangulate(SgPolygonMesh* polygonMesh);
    bool setIndices(
        SgIndexArray& indices, int numElements,
        const SgIndexArray& orgIndices, const SgIndexArray& orgPolygonVertices, int elementTypeId);
};
}


PolygonMeshTriangulator::PolygonMeshTriangulator()
{
    impl = new PolygonMeshTriangulatorImpl();
}


PolygonMeshTriangulator::PolygonMeshTriangulator(const PolygonMeshTriangulator& org)
{
    impl = new PolygonMeshTriangulatorImpl(*org.impl);
}


PolygonMeshTriangulator::~PolygonMeshTriangulator()
{
    delete impl;
}


PolygonMeshTriangulatorImpl::PolygonMeshTriangulatorImpl()
{
    isDeepCopyEnabled = false;
}


void PolygonMeshTriangulator::setDeepCopyEnabled(bool on)
{
    impl->isDeepCopyEnabled = on;
}


const std::string& PolygonMeshTriangulator::errorMessage() const
{
    return impl->errorMessage;
}


SgMesh* PolygonMeshTriangulator::triangulate(SgPolygonMesh* polygonMesh)
{
    return impl->triangulate(polygonMesh);
}


SgMesh* PolygonMeshTriangulatorImpl::triangulate(SgPolygonMesh* orgMesh)
{
    errorMessage.clear();
    
    const SgIndexArray& polygonVertices = orgMesh->polygonVertices();

    if(!orgMesh->vertices() || polygonVertices.empty()){
        return SgMeshPtr();
    }
        
    SgMesh* mesh = new SgMesh();
    if(isDeepCopyEnabled){
        mesh->setVertices(new SgVertexArray(*orgMesh->vertices()));
    } else {
        mesh->setVertices(orgMesh->vertices());
    }
    const SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    triangulator.setVertices(vertices);
    SgIndexArray& triangleVertices = mesh->triangleVertices();

    polygon.clear();
    newIndexPositionToOrgPositionWithDelimitersMap.clear();
    newIndexPositionToOrgPositionMap.clear();
    int polygonTopIndexPosition = 0;
    int polygonTopIndexPositionWithoutDelimiters = 0;
    int numInvalidIndices = 0;
    
    for(size_t i=0; i < polygonVertices.size(); ++i){
        const int index = polygonVertices[i];
        if(index >= numVertices){
            if(numInvalidIndices == 0){
                addErrorMessage(fmt::format("Vertex index {0} is over the number of vertices ({1}).", index, numVertices));
            }
            ++numInvalidIndices;
        } else if(index >= 0){
            polygon.push_back(index);
        } else {
            if(polygon.size() == 3){
                for(int j=0; j < 3; ++j){
                    triangleVertices.push_back(polygon[j]);
                    newIndexPositionToOrgPositionWithDelimitersMap.push_back(polygonTopIndexPosition + j);
                    newIndexPositionToOrgPositionMap.push_back(polygonTopIndexPositionWithoutDelimiters + j);
                }
            } else {
                const int numTriangles = triangulator.apply(polygon);
                const vector<int>& triangles = triangulator.triangles();
                for(int j=0; j < numTriangles; ++j){
                    for(int k=0; k < 3; ++k){
                        int localIndex = triangles[j * 3 + k];
                        triangleVertices.push_back(polygon[localIndex]);
                        newIndexPositionToOrgPositionWithDelimitersMap.push_back(polygonTopIndexPosition + localIndex);
                        newIndexPositionToOrgPositionMap.push_back(polygonTopIndexPositionWithoutDelimiters + localIndex);
                    }
                }
            }
            polygonTopIndexPosition = i + 1;
            polygonTopIndexPositionWithoutDelimiters += polygon.size();
            polygon.clear();
        }
    }

    if(numInvalidIndices > 1){
        addErrorMessage(fmt::format("There are {0} invalied vertex indices that are over the number of vertices ({1}).",
                numInvalidIndices, numVertices));
    }
    if(mesh->numTriangles() == 0){
        addErrorMessage("There is no valid polygons to triangulete.");
        delete mesh;
        return 0;
    }

    SgNormalArray* normals = orgMesh->normals();
    if(normals && !normals->empty()){
        if(setIndices(mesh->normalIndices(), normals->size(), orgMesh->normalIndices(), polygonVertices, 0)){
            if(isDeepCopyEnabled){
                mesh->setNormals(new SgNormalArray(*normals));
            } else {
                mesh->setNormals(normals);
            }
        }
    }

    SgColorArray* colors = orgMesh->colors();
    if(colors && !colors->empty()){
        if(setIndices(mesh->colorIndices(), colors->size(), orgMesh->colorIndices(), polygonVertices, 1)){
            if(isDeepCopyEnabled){
                mesh->setColors(new SgColorArray(*colors));
            } else {
                mesh->setColors(colors);
            }
        }
    }

    SgTexCoordArray* texCoords = orgMesh->texCoords();
    if(texCoords && !texCoords->empty()){
        if(setIndices(mesh->texCoordIndices(), texCoords->size(), orgMesh->texCoordIndices(), polygonVertices, 2)){
            if(isDeepCopyEnabled){
                mesh->setTexCoords(new SgTexCoordArray(*texCoords));
            } else {
                mesh->setTexCoords(texCoords);
            }
        }
    }

    mesh->setSolid(orgMesh->isSolid());
    mesh->updateBoundingBox();

    return mesh;
}


namespace {
const char* message1(int elementTypeId){
    switch(elementTypeId){
    case 0: return "The number of normals is less than the number of vertices.";
    case 1: return "The number of colors is less than the number of vertices.";
    case 2: return "The number of texCoords is less than the number of vertices.";
    default: return 0;
    }
}

const char* message2(int elementTypeId){
    switch(elementTypeId){
    case 0: return "The number of normal indices is less than the number of polygon vertices.";
    case 1: return "The number of color indices is less than the number of polygon vertices.";
    case 2: return "The number of texCoord indices is less than the number of polygon vertices.";
    default: return 0;
    }
}

const char* message3(int elementTypeId){
    switch(elementTypeId){
    case 0: return "Normal index {} is over the range of given normals.";
    case 1: return "Color index {} is over the range of given colors.";
    case 2: return "TexCoord index {} is over the range of given texCoords.";
    default: return 0;
    }
}
}        
          

bool PolygonMeshTriangulatorImpl::setIndices
(SgIndexArray& indices, int numElements, const SgIndexArray& orgIndices, const SgIndexArray& orgPolygonVertices, int elementTypeId)
{
    bool result = true;
    const int numNewIndices = newIndexPositionToOrgPositionMap.size();
    indices.resize(numNewIndices);

    if(orgIndices.empty()){
        for(int i=0; i < numNewIndices; ++i){
            const int index = orgPolygonVertices[newIndexPositionToOrgPositionWithDelimitersMap[i]];
            if(index >= numElements){
                addErrorMessage(message1(elementTypeId));
                result = false;
                break;
            }
            indices[i] = index;
        }
    } else {
        const int numOrgIndices = orgIndices.size();
        for(int i=0; i < numNewIndices; ++i){
            const int orgPos = newIndexPositionToOrgPositionWithDelimitersMap[i];
            if(orgPos >= numOrgIndices){
                addErrorMessage(message2(elementTypeId));
                result = false;
                break;
            }
            const int index = orgIndices[orgPos];
            if(index < 0 || index >= numElements){
                addErrorMessage(fmt::format(message3(elementTypeId), index));
                result = false;
                break;
            }
            indices[i] = index;
        }
    }
    
    if(!result){
        indices.clear();
    }
    return result;
}
