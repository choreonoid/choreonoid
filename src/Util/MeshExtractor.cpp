/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshExtractor.h"
#include "SceneDrawables.h"
#include <functional>

using namespace cnoid;


void MeshExtractor::visitPosTransform(SgPosTransform* transform)
{
    const Affine3 T0(currentTransform_);
    const Affine3 P0(currentTransformWithoutScaling_);
    currentTransform_ = T0 * transform->T();
    currentTransformWithoutScaling_ = P0 * transform->T();
    visitGroup(transform);
    currentTransform_ = T0;
    currentTransformWithoutScaling_ = P0;
}


void MeshExtractor::visitScaleTransform(SgScaleTransform* transform)
{
    bool isParentScaled = isCurrentScaled_;
    isCurrentScaled_ = true;
    Affine3 T0 = currentTransform_;
    currentTransform_ = T0 * transform->T();
    visitGroup(transform);
    currentTransform_ = T0;
    isCurrentScaled_ = isParentScaled;
}


void MeshExtractor::visitShape(SgShape* shape)
{
    SgMesh* mesh = shape->mesh();
    if(mesh && mesh->vertices() && !mesh->vertices()->empty() && !mesh->triangleVertices().empty()){
        meshFound = true;
        currentMesh_ = mesh;
        callback();
        currentMesh_ = 0;
    }
}


void MeshExtractor::visitPointSet(SgPointSet* pointSet)
{

}


void MeshExtractor::visitLineSet(SgLineSet* lineSet)
{

}


void MeshExtractor::visitLight(SgLight* light)
{

}


void MeshExtractor::visitCamera(SgCamera* camera)
{

}


bool MeshExtractor::extract(SgNode* node, std::function<void()> callback)
{
    this->callback = callback;
    currentMesh_ = 0;
    currentTransform_.setIdentity();
    currentTransformWithoutScaling_.setIdentity();
    isCurrentScaled_ = false;
    meshFound = false;
    node->accept(*this);
    return meshFound;
}


static void integrateMesh(MeshExtractor* extractor, SgMesh* mesh)
{
    SgMesh* srcMesh = extractor->currentMesh();
    const Affine3f T = extractor->currentTransform().cast<Affine3f::Scalar>();

    if(srcMesh->hasVertices()){
        SgVertexArray& vertices = *mesh->getOrCreateVertices();
        const int numVertices = vertices.size();
        SgVertexArray& srcVertices = *srcMesh->vertices();
        const int numSrcVertices = srcVertices.size();
        vertices.reserve(numVertices + numSrcVertices);
        for(int i=0; i < numSrcVertices; ++i){
            vertices.push_back(T * srcVertices[i]);
        }

        SgIndexArray& indices = mesh->triangleVertices();
        const int numIndices = indices.size();
        SgIndexArray& srcIndices = srcMesh->triangleVertices();
        const int numSrcIndices = srcIndices.size();
        indices.reserve(numIndices + numSrcIndices);
        for(int i=0; i < numSrcIndices; ++i){
            indices.push_back(srcIndices[i] + numVertices);
        }
        
        if(srcMesh->hasNormals()){
            SgNormalArray& normals = *mesh->getOrCreateNormals();
            const int numNormals = normals.size();
            SgNormalArray& srcNormals = *srcMesh->normals();
            const int numSrcNormals = srcNormals.size();
            normals.reserve(numNormals + numSrcNormals);
            const Affine3f U = extractor->currentTransformWithoutScaling().cast<Affine3f::Scalar>();
            for(int i=0; i < numSrcNormals; ++i){
                normals.push_back(U * srcNormals[i]);
            }
            
            SgIndexArray& indices = mesh->normalIndices();
            const int numIndices = indices.size();
            SgIndexArray& srcIndices = srcMesh->normalIndices();
            const int numSrcIndices = srcIndices.size();
            indices.reserve(numIndices + numSrcIndices);
            for(int i=0; i < numSrcIndices; ++i){
                indices.push_back(srcIndices[i] + numNormals);
            }
        }
    }
}


/**
   \todo take into acount the case where some meshes have normals or colors
   and others don't have them.
*/
SgMesh* MeshExtractor::integrate(SgNode* node)
{
    SgMesh* mesh = new SgMesh;
    extract(node, std::bind(integrateMesh, this, mesh));
    return mesh;
}
