/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneVisitor.h"
#include "SceneGraph.h"
#include "SceneDrawables.h"
#include "SceneCameras.h"
#include "SceneLights.h"
#include "SceneEffects.h"

using namespace cnoid;


SceneVisitor::SceneVisitor()
{
    property_ = new Mapping();
}


SceneVisitor::~SceneVisitor()
{

}


void SceneVisitor::visitNode(SgNode* node)
{

}


void SceneVisitor::visitGroup(SgGroup* group)
{
    visitNode(group);

    for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
        (*p)->accept(*this);
    }
}


void SceneVisitor::visitTransform(SgTransform* transform)
{
    visitGroup(transform);
}


void SceneVisitor::visitPosTransform(SgPosTransform* transform)
{
    visitTransform(transform);
}


void SceneVisitor::visitScaleTransform(SgScaleTransform* transform)
{
    visitTransform(transform);
}


void SceneVisitor::visitInvariantGroup(SgInvariantGroup* group)
{
    visitGroup(group);
}


void SceneVisitor::visitSwitch(SgSwitch* switchNode)
{
    if(switchNode->isTurnedOn()){
        visitGroup(switchNode);
    }
}


void SceneVisitor::visitUnpickableGroup(SgUnpickableGroup* group)
{
    visitGroup(group);
}


void SceneVisitor::visitShape(SgShape* shape)
{
    visitNode(shape);
}


void SceneVisitor::visitPlot(SgPlot* plot)
{
    visitNode(plot);
}


void SceneVisitor::visitPointSet(SgPointSet* pointSet)
{
    visitNode(pointSet);
}


void SceneVisitor::visitLineSet(SgLineSet* lineSet)
{
    visitNode(lineSet);
}


void SceneVisitor::visitPreprocessed(SgPreprocessed* preprocessed)
{
    visitNode(preprocessed);
}


void SceneVisitor::visitLight(SgLight* light)
{
    visitPreprocessed(light);
}


void SceneVisitor::visitFog(SgFog* fog)
{
    visitPreprocessed(fog);
}


void SceneVisitor::visitCamera(SgCamera* camera)
{
    visitPreprocessed(camera);
}


void SceneVisitor::visitOverlay(SgOverlay* overlay)
{

}


void SceneVisitor::visitOutlineGroup(SgOutlineGroup* outline)
{
    visitGroup(outline);
}
