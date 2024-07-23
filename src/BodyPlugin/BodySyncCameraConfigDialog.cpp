#include "BodySyncCameraConfigDialog.h"
#include "BodyPlugin.h"
#include "BodyItem.h"
#include "BodySyncCameraItem.h"
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneCameras>
#include <cnoid/EigenUtil>
#include <cnoid/ConnectionSet>
#include <cnoid/LineEdit>
#include <cnoid/ButtonGroup>
#include <cnoid/Buttons>
#include <cnoid/DoubleSpinBox>
#include <cnoid/LengthSpinBox>
#include <cnoid/CheckBox>
#include <cnoid/Format>
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace cnoid {

class BodySyncCameraConfigDialog::Impl
{
public:
    BodySyncCameraConfigDialog* self;
    BodySyncCameraItemPtr cameraItem;
    ButtonGroup coordinateRadioGroup;
    RadioButton globalRadio;
    RadioButton localRadio;
    CheckBox parallelTrackingCheck;
    ConnectionSet widgetConnections;

    Impl(BodySyncCameraConfigDialog* self);
    void alignWithTargetOrigin();    
    void onParallelTrackingModeToggled(bool on);
};

}


BodySyncCameraConfigDialog* BodySyncCameraConfigDialog::instance()
{
    static BodySyncCameraConfigDialog* instance_ = nullptr;
    if(!instance_){
        instance_ = new BodySyncCameraConfigDialog;
        BodyPlugin::instance()->manage(instance_);
    }
    return instance_;
}





BodySyncCameraConfigDialog::BodySyncCameraConfigDialog()
{
    impl = new Impl(this);
}


BodySyncCameraConfigDialog::Impl::Impl(BodySyncCameraConfigDialog* self_)
    : self(self_)
{
    auto alignButton = new PushButton(_("Align with the target origin"));
    alignButton->sigClicked().connect(
        [&](){ alignWithTargetOrigin(); });
    self->alignVBox()->addWidget(alignButton);
    
    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Coordinate System:")));
    globalRadio.setText(_("Global"));
    hbox->addWidget(&globalRadio);
    localRadio.setText(_("Local"));
    localRadio.setChecked(true);
    hbox->addWidget(&localRadio);
    hbox->addStretch();
    self->optionVBox1()->addLayout(hbox);

    coordinateRadioGroup.addButton(&globalRadio, 0);
    coordinateRadioGroup.addButton(&localRadio, 1);
    coordinateRadioGroup.sigButtonToggled().connect(
        [this](int id, bool checked){
            if(checked) self->updateWidgetsWithCurrentCameraStates();
        });

    parallelTrackingCheck.setText(_("Parallel tracking"));
    widgetConnections.add(
        parallelTrackingCheck.sigToggled().connect(
            [this](bool on){ onParallelTrackingModeToggled(on); }));
    self->optionVBox2()->addWidget(&parallelTrackingCheck);
}

    
BodySyncCameraConfigDialog::~BodySyncCameraConfigDialog()
{
    delete impl;
}


BodySyncCameraItem* BodySyncCameraConfigDialog::showToCreateCameraItem(BodyItem* bodyItem, Link* link)
{
    setWindowTitle(_("Camera Creation"));
    
    BodySyncCameraItemPtr cameraItem = new BodySyncCameraItem;
    cameraItem->setName(formatR(_("{0} Camera"), bodyItem->name()));
    cameraItem->setChecked(true);
    cameraItem->setParallelTrackingMode(false);
    cameraItem->setInteractiveViewpointChangeEnabled(false);

    auto body = bodyItem->body();
    if(link){
        cameraItem->setTargetLink(link->name());
    } else {
        link = body->rootLink();
    }
    cameraItem->cameraTransform()->setPosition(
        link->T() * AngleAxis(PI / 2.0, Vector3::UnitZ()) * AngleAxis(PI, Vector3::UnitX()));

    bodyItem->addChildItem(cameraItem);

    showToConfigureCameraItem(cameraItem);

    return cameraItem;
}


void BodySyncCameraConfigDialog::showToConfigureCameraItem(CameraItem* cameraItem)
{
    impl->cameraItem = dynamic_cast<BodySyncCameraItem*>(cameraItem);
    if(impl->cameraItem){
        CameraConfigDialog::showToConfigureCameraItem(cameraItem);
    }
}


void BodySyncCameraConfigDialog::updateWidgetsWithCurrentCameraStates()
{
    impl->cameraItem->updateRelativeCameraPosition();
    
    impl->widgetConnections.block();
    impl->parallelTrackingCheck.setChecked(impl->cameraItem->isParallelTrackingMode());
    impl->widgetConnections.unblock();

    CameraConfigDialog::updateWidgetsWithCurrentCameraStates();
    
}


Isometry3 BodySyncCameraConfigDialog::getCurrentCameraPositionToDisplay()
{
    Isometry3 T;
    if(impl->globalRadio.isChecked()){
        T = impl->cameraItem->cameraTransform()->T();
    } else {
        T = impl->cameraItem->relativeCameraPosition();
    }
    return T;
}


void BodySyncCameraConfigDialog::setCameraPositionToDisplayToCameraTransform(const Isometry3& T)
{
    Isometry3 Tg;
    if(impl->localRadio.isChecked()){
        Tg = impl->cameraItem->targetLinkPosition() * T;
    } else {
        Tg = T;
    }
    auto transform = impl->cameraItem->cameraTransform();
    transform->setPosition(Tg);
    transform->notifyUpdate();
}


void BodySyncCameraConfigDialog::Impl::alignWithTargetOrigin()
{
    auto transform = cameraItem->cameraTransform();
    transform->setPosition(cameraItem->targetLinkPosition());
    transform->notifyUpdate();
}
    

void BodySyncCameraConfigDialog::Impl::onParallelTrackingModeToggled(bool on)
{
    cameraItem->setParallelTrackingMode(on);
    cameraItem->notifyUpdate();
}
