/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_ITEM_H
#define CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_ITEM_H

#include <cnoid/Item>
#include <cnoid/RenderableItem>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {

class SgCamera;
class SgPerspectiveCamera;
class SgOrthographicCamera;
class SgPosTransform;
class BodyItem;
class Link;

class CNOID_EXPORT BodySyncCameraItem : public Item, public RenderableItem
{
public:
    static void initializeClass(ExtensionManager* ext);
    static BodySyncCameraItem* showDialogToCreateBodySyncCameraItem(BodyItem* bodyItem, Link* link);

    enum CameraType {
        Perspective,
        Orthographic,
        NumCameraTypes,
        // deprecated
        PERSPECTIVE = Perspective,
        ORTHOGRAPHIC = Orthographic,
        N_CAMERA_TYPE
    };

    BodySyncCameraItem();
    BodySyncCameraItem(const BodySyncCameraItem& org);
    ~BodySyncCameraItem();

    virtual bool setName(const std::string& name) override;

    void setTargetLink(const std::string& name);
    const std::string& targetLinkName() const;
    Isometry3 targetLinkPosition() const;
    void setParallelTrackingMode(bool on);
    bool isParallelTrackingMode() const;
    void setInteractiveViewpointChangeEnabled(bool on);
    bool isInteractiveViewpointChangeEnabled() const;
    void setCameraType(CameraType type);
    CameraType cameraType() const;
    SgPerspectiveCamera* perspectiveCamera();
    SgOrthographicCamera* orthographicCamera();
    SgCamera* currentCamera();
    SgPosTransform* cameraTransform();
    Isometry3 relativeCameraPosition() const;

    double fieldOfView() const;
    bool setFieldOfView(double fov);

    double nearClipDistance() const;
    bool setNearClipDistance(double distance);
    double farClipDistance() const;
    bool setFarClipDistance(double distance);
    bool setClipDistances(double nearDistance, double farDistance);

    void showDialogToConfigureCamera();

    // RenderableItem
    virtual SgNode* getScene() override;

    [[deprecated("Use setParallelTrackingMode.")]]
    void setRotationSyncEnabled(bool on) {
        setParallelTrackingMode(!on);
    }
    [[deprecated("Use isParallelTrackingMode.")]]
    bool isRotationSyncEnabled() const {
        return !isParallelTrackingMode();
    }

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

typedef ref_ptr<BodySyncCameraItem> BodySyncCameraItemPtr;

// for the backward compatibility
[[deprecated]]
typedef BodySyncCameraItem BodyTrackingCameraItem;
[[deprecated]]
typedef ref_ptr<BodySyncCameraItem> BodyTrackingCameraItemPtr;

}

#endif
