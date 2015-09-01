/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_VISITOR_H
#define CNOID_UTIL_SCENE_VISITOR_H

#include "SceneGraph.h"
#include "ValueTree.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneVisitor
{
public:
    SceneVisitor();
    virtual ~SceneVisitor();
    virtual void visitNode(SgNode* node);
    virtual void visitGroup(SgGroup* group);
    virtual void visitInvariantGroup(SgInvariantGroup* group);
    virtual void visitTransform(SgTransform* transform);
    virtual void visitPosTransform(SgPosTransform* transform);
    virtual void visitScaleTransform(SgScaleTransform* transform);
    virtual void visitSwitch(SgSwitch* switchNode);
    virtual void visitUnpickableGroup(SgUnpickableGroup* group);
    virtual void visitShape(SgShape* shape);
    virtual void visitPlot(SgPlot* plot);
    virtual void visitPointSet(SgPointSet* pointSet);        
    virtual void visitLineSet(SgLineSet* lineSet);        
    virtual void visitPreprocessed(SgPreprocessed* preprocessed);
    virtual void visitLight(SgLight* light);
    virtual void visitCamera(SgCamera* camera);
    virtual void visitOverlay(SgOverlay* overlay);
    virtual void visitOutlineGroup(SgOutlineGroup* outline);

    Mapping* property() { return property_.get(); }

private:
    MappingPtr property_;
};

}

#endif
