#ifndef CNOID_UTIL_SCENE_NODE_EXTRACTOR_H
#define CNOID_UTIL_SCENE_NODE_EXTRACTOR_H

#include "SceneGraph.h"
#include <functional>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneNodeExtractor
{
public:
    SgNodePath extractNode(SgNode* root, std::function<bool(SgNode* node)> pred, bool includeRoot = true){
        SgNodePath nodePath;
        extractNode_(root, pred, includeRoot, nodePath);
        return nodePath;
    }

    template<class NodeType>
    SgNodePath extractNode(SgNode* root, bool includeRoot = true){
        SgNodePath nodePath;
        extractNode_(
            root,
            [](SgNode* node){
                if(dynamic_cast<NodeType*>(node)){
                    return true;
                }
                return false;
            },
            includeRoot,
            nodePath
            );
        return nodePath;
    }

    template<class NodeType>
    std::vector<SgNodePath> extractNodes(SgNode* root, bool includeRoot = true){
        std::vector<SgNodePath> nodePaths;
        extractNodes_(
            root,
            [](SgNode* node){
                if(dynamic_cast<NodeType*>(node)){
                    return true;
                }
                return false;
            },
            includeRoot,
            nodePaths);
        return nodePaths;
    }

private:
    static void extractNode_(
        SgNode* root, const std::function<bool(SgNode* node)>& pred, bool includeRoot, SgNodePath& nodePath);
    static void extractNodes_(
        SgNode* root, const std::function<bool(SgNode* node)>& pred, bool includeRoot, std::vector<SgNodePath>& nodePaths);
};

}

#endif
