/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_TRACKING_CAMERA_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_TRACKING_CAMERA_ITEM_H

#include <cnoid/Item>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class SgCamera;
class SgPosTransform;

class CNOID_EXPORT BodyTrackingCameraItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    
    enum CameraType {
        Perspective,
        Orthographic,
        NumCameraTypes,
        // deprecated
        PERSPECTIVE = Perspective,
        ORTHOGRAPHIC = Orthographic,
        N_CAMERA_TYPE
    };

    BodyTrackingCameraItem();
    BodyTrackingCameraItem(const BodyTrackingCameraItem& org);
    ~BodyTrackingCameraItem();

    virtual bool setName(const std::string& name) override;

    void setTargetLink(const std::string& name);
    const std::string& targetLinkName() const;
    void setRotationSyncEnabled(bool on);
    bool isRotationSyncEnabled() const;
    void setCameraType(CameraType type);
    CameraType cameraType() const;
    SgCamera* currentCamera();
    SgPosTransform* cameraTransform();

    // RenderableItem
    virtual SgNode* getScene() override;

protected:
    virtual Item* doDuplicate() const override;
    virtual void onTreePathChanged() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    
private:
    class Impl;
    Impl* impl;
};
  
typedef ref_ptr<BodyTrackingCameraItem> BodyTrackingCameraItemPtr;

}

#endif
