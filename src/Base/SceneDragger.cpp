/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneDragger.h"
#include <cnoid/SceneUtil>
#include <cnoid/EigenUtil>

using namespace std;
using namespace cnoid;

namespace {

const char* axisNames[3] = { "x", "y", "z" };

}


SceneDragger::SceneDragger()
{
    isContainerMode_ = false;
}


SceneDragger::SceneDragger(const SceneDragger& org, SgCloneMap* cloneMap)
    : SgPosTransform(org, cloneMap)
{
    isContainerMode_ = org.isContainerMode_;
}


void SceneDragger::setContainerMode(bool on)
{
    isContainerMode_ = on;
}


bool SceneDragger::detectAxisFromNodePath
(const SgNodePath& path, SgNode* topNode, int& out_axis, int& out_indexOfTopNode)
{
    for(size_t i=0; i < path.size(); ++i){
        const SgNode* node = path[i];
        if(node == topNode){
            out_indexOfTopNode = i;
        }
        if(out_indexOfTopNode >= 0){
            const string& name = node->name();
            if(!name.empty()){
                for(int j=0; j < 3; ++j){
                     if(name == axisNames[j]){
                        out_axis = j;
                        return true;
                    }
                }
            }
        }
    }
    return false;
}
