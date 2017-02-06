/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_PROCESSOR_H
#define CNOID_UTIL_SCENE_PROCESSOR_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneProcessor
{
public:
    typedef std::function<void(SceneProcessor* proc, SgNode* node)> NodeFunction;

private:
    std::vector<NodeFunction> dispatchTable;;
    std::vector<bool> dispatchTableFixedness;
    bool isDispatchTableDirty;

    void doUpdateDispatchTable();
    
public:
    SceneProcessor();
    
    template <class ProcessorType, class NodeType>
        void setFunction(std::function<void(ProcessorType* proc, NodeType* node)> func){
        int number = SgNode::findTypeNumber<NodeType>();
        if(number){
            if(number >= dispatchTable.size()){
                dispatchTable.resize(number + 1);
                dispatchTableFixedness.resize(number + 1, false);
            }
            dispatchTable[number] = [func](SceneProcessor* proc, SgNode* node){
                func(static_cast<ProcessorType*>(proc), static_cast<NodeType*>(node));
            };
            dispatchTableFixedness[number] = true;
            isDispatchTableDirty = true;
        }
    }

    void updateDispatchTable() {
        if(isDispatchTableDirty){
            doUpdateDispatchTable();
        }
    }

    void dispatch(SgNode* node){
        NodeFunction& func = dispatchTable[node->typeNumber()];
        if(func){
            func(this, node);
        }
    }

    template <class NodeType>
    void process(SgNode* node){
        NodeFunction& func = dispatchTable[SgNode::findTypeNumber<NodeType>()];
        if(func){
            func(this, node);
        }
    }

    /*
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
    virtual void visitFog(SgFog* fog);
    virtual void visitCamera(SgCamera* camera);
    virtual void visitOverlay(SgOverlay* overlay);
    virtual void visitOutlineGroup(SgOutlineGroup* outline);
    */
};


}

#endif
