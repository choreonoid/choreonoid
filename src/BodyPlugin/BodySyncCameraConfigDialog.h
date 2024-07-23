#ifndef CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_CONFIG_DIALOG_H
#define CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_CONFIG_DIALOG_H

#include <cnoid/CameraConfigDialog>

namespace cnoid {

class BodyItem;
class BodySyncCameraItem;
class Link;

class BodySyncCameraConfigDialog : public CameraConfigDialog
{
public:
    BodySyncCameraConfigDialog();
    virtual ~BodySyncCameraConfigDialog();

    static BodySyncCameraConfigDialog* instance();

    BodySyncCameraItem* showToCreateCameraItem(BodyItem* bodyItem, Link* link);
    virtual void showToConfigureCameraItem(CameraItem* cameraItem) override;

protected:
    virtual void updateWidgetsWithCurrentCameraStates() override;
    virtual Isometry3 getCurrentCameraPositionToDisplay() override;
    virtual void setCameraPositionToDisplayToCameraTransform(const Isometry3& T) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
