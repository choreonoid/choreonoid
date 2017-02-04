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
    std::vector<NodeFunction> functions;;
    
public:
    SceneProcessor();
    
    template <class ProcessorType, class NodeType>
        void setFunction(std::function<void(ProcessorType* proc, NodeType* node)> func){
        int number = SgNode::findTypeNumber<NodeType>();
        if(number){
            if(functions.size() >= number){
                functions.resize(SgNode::numRegistredTypes());
            }
            functions[number] = [func](SceneProcessor* proc, SgNode* node){
                func(static_cast<ProcessorType*>(proc), static_cast<NodeType*>(node));
            };
        }
    }

    void complementDispatchTable();

    void dispatch(SgNode* node){
        functions[node->typeNumber()](this, node);
    }

    template <class NodeType>
    void process(SgNode* node){
        functions[SgNode::findTypeNumber<NodeType>()](this, node);
    }
};

}

#endif
