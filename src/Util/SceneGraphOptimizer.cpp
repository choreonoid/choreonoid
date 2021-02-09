#include "SceneGraphOptimizer.h"
#include "SceneGraph.h"
#include "SceneDrawables.h"
#include "CloneMap.h"
#include <unordered_map>
#include <deque>

using namespace std;
using namespace cnoid;

namespace cnoid {

class SceneGraphOptimizer::Impl
{
public:
    typedef deque<SgNode*> NodePath;

    struct MeshInfo {
        vector<NodePath> paths;
        NodePath commonPath;
    };
    
    unordered_map<SgMesh*, MeshInfo> meshInfoMap;

    int optimizedPathCounter;

    int simplifyTransformPathsWithTransformedMeshes(SgGroup* scene, CloneMap& cloneMap);
    void extractMeshPaths(SgGroup* group, NodePath& path);
    void extractCommonPathsToMeshes();
    void transformMeshes();
    void simplifyMeshPaths(CloneMap& cloneMap);
};

}


SceneGraphOptimizer::SceneGraphOptimizer()
{
    impl = new Impl;
}


SceneGraphOptimizer::~SceneGraphOptimizer()
{
    delete impl;
}


int SceneGraphOptimizer::simplifyTransformPathsWithTransformedMeshes(SgGroup* scene, CloneMap& cloneMap)
{
    return impl->simplifyTransformPathsWithTransformedMeshes(scene, cloneMap);
}


int SceneGraphOptimizer::Impl::simplifyTransformPathsWithTransformedMeshes(SgGroup* scene, CloneMap& cloneMap)
{
    optimizedPathCounter = 0;

    NodePath path;
    extractMeshPaths(scene, path);
    extractCommonPathsToMeshes();
    simplifyMeshPaths(cloneMap);
    
    meshInfoMap.clear();

    return optimizedPathCounter;
}


void SceneGraphOptimizer::Impl::extractMeshPaths(SgGroup* group, NodePath& path)
{
    path.push_back(group);
    
    for(auto& node : *group){
        if(node->isGroupNode()){
            extractMeshPaths(node->toGroupNode(), path);
        } else if(auto shape = dynamic_cast<SgShape*>(node.get())){
            auto mesh = shape->mesh();
            if(mesh && mesh->hasVertices() && mesh->primitiveType() == SgMesh::MeshType){
                auto& info = meshInfoMap[mesh];
                info.paths.push_back(path);
                info.paths.back().push_back(shape);
                int i=0;
            }
        }
    }

    path.pop_back();
}


void SceneGraphOptimizer::Impl::extractCommonPathsToMeshes()
{
    auto iter = meshInfoMap.begin();
    
    while(iter != meshInfoMap.end()){

        auto& info = iter->second;
        auto& paths = info.paths;
        info.commonPath = paths.front();
        auto& commonPath = info.commonPath;

        // Remove the shape node
        commonPath.pop_back();
        
        // Top group must always be kept to contain the remaining part
        commonPath.pop_front();
        
        for(size_t i=2; i < paths.size(); ++i){
            auto& anotherPath = paths[i];
            size_t j = commonPath.size();
            size_t k = anotherPath.size() - 1;
            while(j >= 0 && k >= 0){
                bool isSameTransform = false;
                auto node1 = commonPath[j - 1];
                auto node2 = anotherPath[k - 1];
                Affine3 T1;
                Affine3 T2;
                if(!node1->isTransformNode()){
                    if(!node2->isTransformNode()){
                        isSameTransform = true;
                    } else {
                        T1.setIdentity();
                        node2->toTransformNode()->getTransform(T2);
                    }
                } else {
                    node1->toTransformNode()->getTransform(T1);
                    if(!node2->isTransformNode()){
                        T2.setIdentity();
                    } else {
                        node2->toTransformNode()->getTransform(T2);
                    }
                }
                if(!isSameTransform){
                    if(T1.isApprox(T2)){
                        isSameTransform = true;
                    }
                }
                if(!isSameTransform){
                    break;
                }
                --j;
                --k;
            }
            for(int l=0; l < j; ++l){
                commonPath.pop_front();
            }
        }
        if(commonPath.empty()){
            iter = meshInfoMap.erase(iter);
        } else {
            ++iter;
        }
    }
}


void SceneGraphOptimizer::Impl::simplifyMeshPaths(CloneMap& cloneMap)
{
    for(auto& kv : meshInfoMap){
        SgMeshPtr mesh = kv.first;
        Affine3 T = Affine3::Identity();
        auto& info = kv.second;
        for(auto& group : info.commonPath){
            if(group->isTransformNode()){
                Affine3 T0;
                group->toTransformNode()->getTransform(T0);
                T = T * T0;
            }
        }
        bool isMeshTransformed = false;
        if(!T.isApprox(Affine3::Identity())){
            mesh = cloneMap.getClone(mesh);
            mesh->transform(T);
            isMeshTransformed = true;
        }

        int depth = info.commonPath.size();
        for(auto& path : info.paths){
            int iShape = path.size() - 1;
            int iOrgParent = iShape - 1;
            int iNewParent = iOrgParent - depth;
            auto shape = static_cast<SgShape*>(path[iShape]);
            if(isMeshTransformed){
                shape->setMesh(mesh);
            }
            auto orgParent = path[iOrgParent]->toGroupNode();
            auto newParent = path[iNewParent]->toGroupNode();
            auto newNext = path[iNewParent + 1];
            newParent->insertChild(newNext, shape);
            orgParent->removeChild(shape);

            // Remove empty group nodes
            for(int i = iOrgParent; i >= iNewParent + 1; --i){
                auto group = path[i]->toGroupNode();
                if(shape->name().empty() && !group->name().empty()){
                    shape->setName(group->name());
                }
                if(group->empty()){
                    path[i-1]->toGroupNode()->removeChild(group);
                }
            }
            
            optimizedPathCounter++;
        }
    }
}
