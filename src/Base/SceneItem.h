/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_ITEM_H
#define CNOID_BASE_SCENE_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include <cnoid/SceneGraph>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SceneItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    SceneItem();
    SceneItem(const SceneItem& org);
    virtual ~SceneItem();

    virtual void setName(const std::string& name) override;

    // RenderableItem
    virtual SgNode* getScene() override;

    SgPosTransform* topNode() { return topNode_; }
    const SgPosTransform* topNode() const { return topNode_; }

    void setTranslation(const Vector3f& translation);
    void setRotation(const AngleAxisf& rotation);

    void setLightweightRenderingEnabled(bool on);
    bool isLightweightRenderingEnabled() const { return isLightweightRenderingEnabled_; }

protected:
    virtual Item* doDuplicate() const override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;

private:
    SgPosTransformPtr topNode_;
    bool isLightweightRenderingEnabled_;

    bool onTranslationChanged(const std::string& value);
    bool onRotationChanged(const std::string& value);
};

typedef ref_ptr<SceneItem> SceneItemPtr;
}
    
#endif
