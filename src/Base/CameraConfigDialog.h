#ifndef CNOID_BASE_CAMERA_CONFIG_DIALOG_H
#define CNOID_BASE_CAMERA_CONFIG_DIALOG_H

#include <cnoid/Dialog>
#include <cnoid/EigenTypes>
#include "exportdecl.h"

class QVBoxLayout;

namespace cnoid {

class Item;
class CameraItem;

class CNOID_EXPORT CameraConfigDialog : public Dialog
{
public:
    CameraConfigDialog();
    virtual ~CameraConfigDialog();

    static CameraConfigDialog* instance();

    CameraItem* showToCreateCameraItem(Item* parentItem);
    virtual void showToConfigureCameraItem(CameraItem* cameraItem);

protected:
    QVBoxLayout* alignVBox();
    QVBoxLayout* optionVBox1();
    QVBoxLayout* optionVBox2();
    CameraItem* cameraItem();
    virtual void updateWidgetsWithCurrentCameraStates();
    virtual Isometry3 getCurrentCameraPositionToDisplay();
    virtual void setCameraPositionToDisplayToCameraTransform(const Isometry3& T);

protected:
    virtual void hideEvent(QHideEvent* event) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
