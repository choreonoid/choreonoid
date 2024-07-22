#ifndef CNOID_BASE_CAMERA_ITEM_H
#define CNOID_BASE_CAMERA_ITEM_H

#include <cnoid/Item>
#include <cnoid/RenderableItem>
#include "exportdecl.h"

namespace cnoid {

class SgCamera;
class SgPerspectiveCamera;
class SgOrthographicCamera;
class SgPosTransform;
class InteractiveCameraTransform;
class SceneView;

class CNOID_EXPORT CameraItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    static CameraItem* showDialogToCreateCameraItem(Item* parentItem);

    enum CameraType {
        Perspective,
        Orthographic,
        NumCameraTypes,
    };

    CameraItem();
    virtual ~CameraItem();

    virtual bool setName(const std::string& name) override;

    void setInteractiveViewpointChangeEnabled(bool on);
    bool isInteractiveViewpointChangeEnabled() const;
    void setCameraType(CameraType type);
    CameraType cameraType() const;
    SgPerspectiveCamera* perspectiveCamera();
    SgOrthographicCamera* orthographicCamera();
    SgCamera* currentCamera();
    SgPosTransform* cameraTransform();

    double fieldOfView() const;
    bool setFieldOfView(double fov);

    double nearClipDistance() const;
    bool setNearClipDistance(double distance);
    double farClipDistance() const;
    bool setFarClipDistance(double distance);
    bool setClipDistances(double nearDistance, double farDistance);

    virtual void showDialogToConfigureCamera();

    void activateCameraInSceneView(SceneView* sceneView, bool on = true);
    void activateSceneViewBuiltinCameraWithCameraItemConfiguration(SceneView* sceneView);

    // RenderableItem
    virtual SgNode* getScene() override;

protected:
    CameraItem(const char* name, InteractiveCameraTransform* cameraTransform);
    CameraItem(const CameraItem& org);
    CameraItem(const CameraItem& org, InteractiveCameraTransform* newCameraTransform);    
    
    virtual Item* doCloneItem(CloneMap* cloneMap) const override;
    virtual void onDoubleClicked() override;
    virtual void doPutProperties(PutPropertyFunction& putProperty) override;
    virtual bool store(Archive& archive) override;
    virtual bool restore(const Archive& archive) override;
    
private:
    class Impl;
    Impl* impl;
};

typedef ref_ptr<CameraItem> CameraItemPtr;

}

#endif
