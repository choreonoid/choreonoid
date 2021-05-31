/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_MARKER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_MARKER_ITEM_H

#include <cnoid/Item>
#include <cnoid/RenderableItem>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class BodyMarkerItemImpl;

class CNOID_EXPORT BodyMarkerItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodyMarkerItem();
    BodyMarkerItem(const BodyMarkerItem& org);
    virtual ~BodyMarkerItem();

    virtual bool setName(const std::string& name) override;

    // RenderableItem
    virtual SgNode* getScene() override;

    bool setTargetLink(const std::string& name);
    bool setTargetNode(const std::string& name);
    void setOffsetPosition(const Isometry3& T);
    
    enum MarkerType {
        CROSS_MARKER,
        SPHERE_MARKER,
        AXES_MARKER,
        N_MARKER_TYPES
    };

    int markerType() const;
    void setMarkerType(int type);
    double markerSize() const;
    void setMarkerSize(double size);
    const Vector3f& markerColor() const;
    void setMarkerColor(const Vector3f& color);

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
            
private:
    BodyMarkerItemImpl* impl;
};

}

#endif
