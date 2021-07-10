#ifndef CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_CONFIG_DIALOG_H
#define CNOID_BODY_PLUGIN_BODY_SYNC_CAMERA_CONFIG_DIALOG_H

#include <cnoid/Dialog>

namespace cnoid {

class BodyItem;
class BodySyncCameraItem;
class Link;

class BodySyncCameraConfigDialog : public Dialog
{
public:
    BodySyncCameraConfigDialog();
    ~BodySyncCameraConfigDialog();

    static BodySyncCameraConfigDialog* instance();

    BodySyncCameraItem* createCameraItem(BodyItem* bodyItem, Link* link);
    void configureCameraItem(BodySyncCameraItem* cameraItem);

protected:
    virtual void hideEvent(QHideEvent* event) override;

private:
    class Impl;
    Impl* impl;
};

}

#endif
