#include "SceneNodeExtractor.h"

using namespace std;
using namespace cnoid;


void SceneNodeExtractor::extractNodes_(SgNode* root, std::function<bool(SgNode* node)> pred)
{
    nodePath.clear();
    nodePaths.clear();
    extractNodesIter(root, pred);
}


void SceneNodeExtractor::extractNodesIter(SgNode* node, const std::function<bool(SgNode* node)>& pred)
{
    nodePath.push_back(node);
    if(pred(node)){
        nodePaths.push_back(nodePath);
    } else if(auto group = dynamic_cast<SgGroup*>(node)){
        for(auto& child : *group){
            extractNodesIter(child, pred);
        }
    }
}
