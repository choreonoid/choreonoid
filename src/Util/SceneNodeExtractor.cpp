#include "SceneNodeExtractor.h"

using namespace std;
using namespace cnoid;


void SceneNodeExtractor::extractNodes_(SgNode* root, std::function<bool(SgNode* node)> pred)
{
    nodePath_.clear();
    nodePathList_.clear();
    extractNodesIter(root, pred);
}


void SceneNodeExtractor::extractNodesIter(SgNode* node, const std::function<bool(SgNode* node)>& pred)
{
    nodePath_.push_back(node);
    if(pred(node)){
        nodePathList_.push_back(nodePath_);
    } else if(auto group = dynamic_cast<SgGroup*>(node)){
        for(auto& child : *group){
            extractNodesIter(child, pred);
        }
    }
}
