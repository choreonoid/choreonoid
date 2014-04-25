/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_ITEM_H_INCLUDED
#define CNOID_BASE_SCENE_ITEM_H_INCLUDED

#include "Item.h"
#include <cnoid/SceneGraph>
#include <cnoid/SceneProvider>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    SceneItem();
    SceneItem(const SceneItem& org);
    virtual ~SceneItem();

    virtual void setName(const std::string& name);
    virtual SgNode* scene();

    SgPosTransform* topNode() { return topNode_.get(); }
    const SgPosTransform* topNode() const { return topNode_.get(); }

protected:
    virtual ItemPtr doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    SgPosTransformPtr topNode_;

    bool onTranslationChanged(const std::string& value);
    bool onRotationChanged(const std::string& value);
};

typedef ref_ptr<SceneItem> SceneItemPtr;
}
    
#endif
