/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneUtil.h"
#include "SceneDrawables.h"
#include "PolymorphicSceneNodeFunctionSet.h"
#include "CloneMap.h"

using namespace std;
using namespace cnoid;

namespace {

void calcTotalTransform
(SgNodePath::const_iterator begin, SgNodePath::const_iterator end, const SgNode* targetNode, Affine3& out_T)
{
    out_T = Affine3::Identity();
    for(SgNodePath::const_iterator p = begin; p != end; ++p){
        SgNode* node = *p;
        SgTransform* transform = dynamic_cast<SgTransform*>(node);
        if(transform){
            Affine3 T;
            transform->getTransform(T);
            out_T = out_T * T;
        }
        if(node == targetNode){
            break;
        }
    }
}
}
    

Affine3 cnoid::calcTotalTransform(const SgNodePath& path)
{
    Affine3 T;
    ::calcTotalTransform(path.begin(), path.end(), nullptr, T);
    return T;
}


Affine3 cnoid::calcTotalTransform(const SgNodePath& path, const SgNode* targetNode)
{
    Affine3 T;
    ::calcTotalTransform(path.begin(), path.end(), targetNode, T);
    return T;
}


Affine3 cnoid::calcTotalTransform(SgNodePath::const_iterator begin, SgNodePath::const_iterator end)
{
    Affine3 T;
    ::calcTotalTransform(begin, end, nullptr, T);
    return T;
}


namespace {

class Transparenter : PolymorphicSceneNodeFunctionSet
{
public:
    CloneMap& cloneMap;
    bool doKeepOrgTransparency;
    float transparency;
    int numModified;
    SgMaterialPtr defaultMaterial;

    Transparenter(CloneMap& cloneMap, bool doKeepOrgTransparency)
        : cloneMap(cloneMap),
          doKeepOrgTransparency(doKeepOrgTransparency)
    {
        setFunction<SgGroup>(
            [&](SgNode* node){ visitGroup(static_cast<SgGroup*>(node)); });
        setFunction<SgShape>(
            [&](SgNode* node){ visitShape(static_cast<SgShape*>(node)); });
        setFunction<SgPlot>(
            [&](SgNode* node){ visitPlot(static_cast<SgPlot*>(node)); });
        updateDispatchTable();
    }

    SgMaterial* getTransparentMaterial(SgMaterial* material)
    {
        SgMaterial* modified = cloneMap.getClone(material);
        if(doKeepOrgTransparency){
            modified->setTransparency(std::max(transparency, material->transparency()));
        } else {
            modified->setTransparency(transparency);
        }
        return modified;
    }
        
    void visitGroup(SgGroup* group)
    {
        for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
            dispatch(*p);
        }
    }
    
    void visitShape(SgShape* shape)
    {
        if(shape->material()){
            shape->setMaterial(getTransparentMaterial(shape->material()));
        } else {
            if(!defaultMaterial){
                defaultMaterial = new SgMaterial;
                defaultMaterial->setTransparency(transparency);
            }
            shape->setMaterial(defaultMaterial);
        }
        ++numModified;
    }
                
    void visitPlot(SgPlot* plot)
    {
        if(plot->material()){
            plot->setMaterial(getTransparentMaterial(plot->material()));
            ++numModified;
        }
    }

    int apply(SgNode* topNode, float transparency)
    {
        this->transparency = transparency;
        numModified = 0;
        dispatch(topNode);
        return numModified;
    }
};

}


int cnoid::makeTransparent(SgNode* topNode, float transparency, CloneMap& cloneMap, bool doKeepOrgTransparency)
{
    if(topNode){
        Transparenter transparenter(cloneMap, doKeepOrgTransparency);
        return transparenter.apply(topNode, transparency);
    }
    return 0;
}
