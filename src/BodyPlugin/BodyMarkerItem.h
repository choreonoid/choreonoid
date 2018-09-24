/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_MARKER_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_MARKER_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneProvider>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class BodyMarkerItemImpl;

class CNOID_EXPORT BodyMarkerItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);
        
    BodyMarkerItem();
    BodyMarkerItem(const BodyMarkerItem& org);
    virtual ~BodyMarkerItem();

    virtual void setName(const std::string& name);
    virtual SgNode* getScene();

    bool setTargetLink(const std::string& name);
    bool setTargetNode(const std::string& name);
    void setOffsetPosition(const Position& T);
    
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
    virtual Item* doDuplicate() const;
    virtual void onPositionChanged();
    virtual void doPutProperties(PutPropertyFunction& putProperty);
    virtual bool store(Archive& archive);
    virtual bool restore(const Archive& archive);
            
private:
    BodyMarkerItemImpl* impl;
};

}

#endif
