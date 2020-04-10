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
    std::vector<SgNodePath> extractNodes(SgNode* root){
        extractNodes_(
            root,
            [](SgNode* node){
                if(dynamic_cast<NodeType*>(node)){
                    return true;
                }
                return false;
            });
        return std::move(nodePathList_);
    }

private:
    void extractNodes_(SgNode* root, std::function<bool(SgNode* node)> pred);
    void extractNodesIter(SgNode* node, const std::function<bool(SgNode* node)>& pred);

    SgNodePath nodePath_;
    std::vector<SgNodePath> nodePathList_;
};

}

#endif
