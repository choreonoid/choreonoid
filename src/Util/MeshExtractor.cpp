/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "MeshExtractor.h"

using namespace std;
using namespace cnoid;


bool MeshExtractor::extract(SgNode* node, boost::function<void()> callback)
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
