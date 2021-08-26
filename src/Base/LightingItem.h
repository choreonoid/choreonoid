#ifndef CNOID_BASE_LIGHTING_ITEM_H
#define CNOID_BASE_LIGHTING_ITEM_H

#include "Item.h"
#include "RenderableItem.h"
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT LightingItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);

    LightingItem();
    LightingItem(const LightingItem& org);
    virtual ~LightingItem();

    enum LightType { DirectionalLight, PointLight, SpotLight, NumLightTypes };

    void setLightType(LightType type);
    void setLightEnabled(bool on);
    void setTranslation(const Vector3& translation);
    void setDirection(const Vector3& direction);
    void setIntensity(float intensity);
    void setAmbientIntensity(float intensity);
    void setColor(const Vector3f& color);
    void setConstantAttenuation(float a0);
    void setLinearAttenuation(float a1);
    void setQuadraticAttenuation(float a2);
    void setBeamWidth(float w);
    void setCutOffAngle(float a);
    void setCutOffExponent(float e);
    void setLightMarkerEnabled(bool on);

protected:
    virtual Item* doDuplicate() const override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;

    // RenderableItem
    virtual SgNode* getScene() override;

private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<LightingItem> LightingItemPtr;

}

#endif
