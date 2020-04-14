#include "SceneNodeExtractor.h"

using namespace std;
using namespace cnoid;


void SceneNodeExtractor::extractNode_(SgNode* root, std::function<bool(SgNode* node)> pred, bool includeRoot)
{
    nodePath_.clear();

    if(includeRoot){
        nodePath_.push_back(root);
    }
    extractNodesIter(root, pred, false);
}


void SceneNodeExtractor::extractNodes_(SgNode* root, std::function<bool(SgNode* node)> pred, bool includeRoot)
{
    nodePath_.clear();
    nodePathList_.clear();

    if(includeRoot){
        nodePath_.push_back(root);
    }
    extractNodesIter(root, pred, true);
}


bool SceneNodeExtractor::extractNodesIter
(SgNode* node, const std::function<bool(SgNode* node)>& pred, bool extractMultiplePaths)
{
    if(pred(node)){
        if(!extractMultiplePaths){
            return true;
        }
        nodePathList_.push_back(nodePath_);

    } else if(auto group = dynamic_cast<SgGroup*>(node)){
        for(auto& child : *group){
            nodePath_.push_back(child);
            if(extractNodesIter(child, pred, extractMultiplePaths)){
                return true;
            }
            nodePath_.pop_back();
        }
    }
    return false;
}
