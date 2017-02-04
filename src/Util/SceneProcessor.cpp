/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#include "SceneProcessor.h"

using namespace std;
using namespace cnoid;

namespace {

void processNode(SceneProcessor* proc, SgNode* node)
{

}

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
    setFunction<SceneProcessor, SgNode>(processNode);
    setFunction<SceneProcessor, SgGroup>(processGroup);
    setFunction<SceneProcessor, SgSwitch>(processSwitch);
}


void SceneProcessor::complementDispatchTable()
{
    const int n = SgNode::numRegistredTypes();
}
