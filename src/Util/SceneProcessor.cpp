/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneProcessor.h"
#include "SceneDrawables.h"
#include "SceneCameras.h"
#include "SceneLights.h"
#include "SceneEffects.h"

using namespace std;
using namespace cnoid;

namespace {

void processGroup(SceneProcessor* proc, SgGroup* group)
{
    for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
        SgNode* child = *p;
        proc->dispatch(child);
    }
}

void processSwitch(SceneProcessor* proc, SgSwitch* switchNode)
{
    if(switchNode->isTurnedOn()){
        processGroup(proc, switchNode);
    }
}

}


SceneProcessor::SceneProcessor()
{
    const int n = SgNode::numRegistredTypes();
    functions.resize(n);
    isFunctionSpecified.resize(n, false);
    
    setFunction<SceneProcessor, SgGroup>(processGroup);
    setFunction<SceneProcessor, SgSwitch>(processSwitch);
}


void SceneProcessor::complementDispatchTable()
{
    const int numRegistredTypes = SgNode::numRegistredTypes();
    if(functions.size() < numRegistredTypes){
        functions.resize(numRegistredTypes);
        isFunctionSpecified.resize(numRegistredTypes, false);
    }
    
    const int n = functions.size();
    for(int i=0; i < n; ++i){
        if(!isFunctionSpecified[i]){
            functions[i] = nullptr;
        }
    }
    for(int i=0; i < n; ++i){
        if(!functions[i]){
            int index = i;
            while(true){
                int superIndex = SgNode::findSuperTypeNumber(index);
                if(superIndex < 0){
                    break;
                }
                if(functions[superIndex]){
                    functions[i] = functions[superIndex];
                    break;
                }
                index = superIndex;
            }
        }
    }
}
