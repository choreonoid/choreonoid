/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_ITEM_H
#define CNOID_BASE_SCENE_ITEM_H

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
    virtual SgNode* getScene();

    SgPosTransform* topNode() { return topNode_; }
    const SgPosTransform* topNode() const { return topNode_; }

    void setTranslation(const Vector3f& translation);
    void setRotation(const AngleAxisf& rotation);

    void setLightweightRenderingEnabled(bool on);
    bool isLightweightRenderingEnabled() const { return isLightweightRenderingEnabled_; }

protected:
    virtual Item* doDuplicate() const;
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
    virtual void doPutProperties(PutPropertyFunction& putProperty);

private:
    SgPosTransformPtr topNode_;
    bool isLightweightRenderingEnabled_;

    bool onTranslationChanged(const std::string& value);
    bool onRotationChanged(const std::string& value);
};

typedef ref_ptr<SceneItem> SceneItemPtr;
}
    
#endif
