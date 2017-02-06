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
    dispatchTable.resize(n);
    dispatchTableFixedness.resize(n, false);
    isDispatchTableDirty = true;
    
    setFunction<SceneProcessor, SgGroup>(processGroup);
    setFunction<SceneProcessor, SgSwitch>(processSwitch);
}


void SceneProcessor::doUpdateDispatchTable()
{
    const int numRegistredTypes = SgNode::numRegistredTypes();
    if(dispatchTable.size() < numRegistredTypes){
        dispatchTable.resize(numRegistredTypes);
        dispatchTableFixedness.resize(numRegistredTypes, false);
    }
    
    const int n = dispatchTable.size();
    for(int i=0; i < n; ++i){
        if(!dispatchTableFixedness[i]){
            dispatchTable[i] = nullptr;
        }
    }
    for(int i=0; i < n; ++i){
        if(!dispatchTable[i]){
            int index = i;
            while(true){
                int superIndex = SgNode::findSuperTypeNumber(index);
                if(superIndex < 0){
                    break;
                }
                if(dispatchTable[superIndex]){
                    dispatchTable[i] = dispatchTable[superIndex];
                    break;
                }
                index = superIndex;
            }
        }
    }
    
    isDispatchTableDirty = false;
}
