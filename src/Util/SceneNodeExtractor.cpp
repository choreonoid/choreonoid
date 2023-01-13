#include "SceneNodeExtractor.h"

using namespace std;
using namespace cnoid;


static bool extractNodeIter
(SgNode* node, const std::function<bool(SgNode* node)>& pred, SgNodePath& nodePath)
{
    if(pred(node)){
        return true;

    } else if(auto group = node->toGroupNode()){
        for(auto& child : *group){
            nodePath.push_back(child);
            if(extractNodeIter(child, pred, nodePath)){
                return true;
            }
            nodePath.pop_back();
        }
    }

    return false;
}


void SceneNodeExtractor::extractNode_
(SgNode* root, const std::function<bool(SgNode* node)>& pred, bool includeRoot, SgNodePath& nodePath)
{
    nodePath.reserve(20);
    if(includeRoot){
        nodePath.push_back(root);
    }
    extractNodeIter(root, pred, nodePath);
}


static void extractNodesIter
(SgNode* node, const std::function<bool(SgNode* node)>& pred, SgNodePath& nodePath, std::vector<SgNodePath>& nodePaths)
{
    if(pred(node)){
        nodePaths.push_back(nodePath);

    } else if(auto group = node->toGroupNode()){
        for(auto& child : *group){
            nodePath.push_back(child);
            extractNodesIter(child, pred, nodePath, nodePaths);
            nodePath.pop_back();
        }
    }
}


void SceneNodeExtractor::extractNodes_
(SgNode* root, const std::function<bool(SgNode* node)>& pred, bool includeRoot, std::vector<SgNodePath>& nodePaths)
{
    nodePaths.reserve(10);
    
    SgNodePath nodePath;
    nodePath.reserve(10);

    if(includeRoot){
        nodePath.push_back(root);
    }

    extractNodesIter(root, pred, nodePath, nodePaths);
}
