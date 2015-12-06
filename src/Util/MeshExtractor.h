/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_MESH_EXTRACTOR_H
#define CNOID_UTIL_MESH_EXTRACTOR_H

#include "SceneVisitor.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT MeshExtractor : public SceneVisitor
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    bool extract(SgNode* node, boost::function<void()> callback);
    SgMesh* integrate(SgNode* node);

    SgMesh* currentMesh() const { return currentMesh_; }
    const Affine3& currentTransform() const { return currentTransform_; }
    const Affine3& currentTransformWithoutScaling() const { return currentTransformWithoutScaling_; }
    bool isCurrentScaled() const { return isCurrentScaled_; }

protected:
    virtual void visitPosTransform(SgPosTransform* transform);
    virtual void visitScaleTransform(SgScaleTransform* transform);
    virtual void visitShape(SgShape* shape);
    virtual void visitPointSet(SgPointSet* pointSet);        
    virtual void visitLineSet(SgLineSet* lineSet);        
    virtual void visitLight(SgLight* light);
    virtual void visitCamera(SgCamera* camera);

private:
    boost::function<void()> callback;
    SgMesh* currentMesh_;
    Affine3 currentTransform_;
    Affine3 currentTransformWithoutScaling_;
    bool isCurrentScaled_;
    bool meshFound;
};

}

#endif
