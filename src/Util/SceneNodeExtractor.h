#ifndef CNOID_UTIL_SCENE_NODE_EXTRACTOR_H
#define CNOID_UTIL_SCENE_NODE_EXTRACTOR_H

#include "SceneGraph.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneNodeExtractor
{
public:
    template<class NodeType>
    SgNodePath extractNode(SgNode* root, bool includeRoot = true){
        extractNode_(
            root,
            [](SgNode* node){
                if(dynamic_cast<NodeType*>(node)){
                    return true;
                }
                return false;
            },
            includeRoot);
        return std::move(nodePath_);
    }

    template<class NodeType>
    std::vector<SgNodePath> extractNodes(SgNode* root, bool includeRoot = true){
        extractNodes_(
            root,
            [](SgNode* node){
                if(dynamic_cast<NodeType*>(node)){
                    return true;
                }
                return false;
            },
            includeRoot);
        return std::move(nodePathList_);
    }

private:
    void extractNode_(SgNode* root, std::function<bool(SgNode* node)> pred, bool includeRoot);
    void extractNodes_(SgNode* root, std::function<bool(SgNode* node)> pred, bool includeRoot);
    bool extractNodesIter(SgNode* node, const std::function<bool(SgNode* node)>& pred, bool extractMultiplePaths);

    SgNodePath nodePath_;
    std::vector<SgNodePath> nodePathList_;
};

}

#endif
