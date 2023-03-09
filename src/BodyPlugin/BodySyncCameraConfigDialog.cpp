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
#include <QBoxLayout>
#include <QLabel>
#include <QDialogButtonBox>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace cnoid {

class BodySyncCameraConfigDialog::Impl
{
public:
    BodySyncCameraConfigDialog* self;
    BodySyncCameraItemPtr cameraItem;
    LineEdit nameEdit;
    ButtonGroup coordinateRadioGroup;
    RadioButton globalRadio;
    RadioButton localRadio;
    LengthSpinBox positionSpins[3];
    DoubleSpinBox directionSpins[3];
    DoubleSpinBox upSpins[3];
    DoubleSpinBox nearClipSpin;
    DoubleSpinBox farClipSpin;
    DoubleSpinBox fovSpin;
    CheckBox parallelTrackingCheck;
    CheckBox interactiveViewpointChangeCheck;
    ConnectionSet widgetConnections;

    CheckBox activationInSceneViewCheck;
    Connection activationCheckConnection;

    ScopedConnection cameraConnection;

    Impl(BodySyncCameraConfigDialog* self);
    void showToConfigureCameraItem(BodySyncCameraItem* cameraItem);
    void setCameraItem(BodySyncCameraItem* cameraItem);
    void updateWidgetsWithCurrentCameraStates();
    void setVectorElementSpins(const Vector3& v, LengthSpinBox spins[]);
    void setVectorElementSpins(const Vector3& v, DoubleSpinBox spins[]);
    void onNameEditingFinished(const std::string& name);
    void alignWithBuiltinCamera();
    void alignWithTargetOrigin();
    void onCameraPositionSpinValueChanged();
    void onFieldOfViewSpinValueChanged(double fov);
    void onClipDistanceSpinValueChanged();
    void onParallelTrackingModeToggled(bool on);
    void onInteractiveViewpointChangeToggled(bool on);
    void onSceneViewActivationChecked(bool on);
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


BodySyncCameraConfigDialog::Impl::Impl(BodySyncCameraConfigDialog* self)
    : self(self)
{
    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Name:")));
    widgetConnections.add(
        nameEdit.sigEditingFinished().connect(
            [&](){ onNameEditingFinished(nameEdit.text().toStdString()); }));
    hbox->addWidget(&nameEdit);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    auto alignVBox = new QVBoxLayout;
    auto alignButton1 = new PushButton(_("Align with the builtin camera"));
    alignButton1->sigClicked().connect(
        [&](){ alignWithBuiltinCamera(); });
    alignVBox->addWidget(alignButton1);
    auto alignButton2 = new PushButton(_("Align with the target origin"));
    alignButton2->sigClicked().connect(
        [&](){ alignWithTargetOrigin(); });
    alignVBox->addWidget(alignButton2);
    hbox->addStretch();
    hbox->addLayout(alignVBox);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Coordinate System:")));
    globalRadio.setText(_("Global"));
    hbox->addWidget(&globalRadio);
    localRadio.setText(_("Local"));
    localRadio.setChecked(true);
    hbox->addWidget(&localRadio);
    hbox->addStretch();
    vbox->addLayout(hbox);

    coordinateRadioGroup.addButton(&globalRadio, 0);
    coordinateRadioGroup.addButton(&localRadio, 1);
    coordinateRadioGroup.sigButtonToggled().connect(
        [this](int id, bool checked){
            if(checked) updateWidgetsWithCurrentCameraStates();
        });

    auto grid = new QGridLayout;
    grid->addWidget(new QLabel(_("Position:")), 0, 0);
    grid->addWidget(new QLabel(_("Direction:")), 1, 0);
    grid->addWidget(new QLabel(_("Up:")), 2, 0);

    DoubleSpinBox* spins[3][3];
    for(int i=0; i < 3; ++i){
        auto spin = &positionSpins[i];
        spin->setMeterRange(-9.999, 9.999);
        spin->setMeterSingleStep(0.001);
        spins[0][i] = spin;
        spins[1][i] = &directionSpins[i];
        spins[2][i] = &upSpins[i];
    }
    for(int i=1; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            auto spin = spins[i][j];
            spin->setDecimals(3);
            spin->setRange(-9.999, 9.999);
            spin->setSingleStep(0.001);
        }
    }
    const char* xyzLabels[] = { "X", "Y", "Z" };
    for(int i=0; i < 3; ++i){
        for(int j=0; j < 3; ++j){
            grid->addWidget(new QLabel(xyzLabels[j]), i, j * 2 + 1);
            auto spin = spins[i][j];
            widgetConnections.add(
                spin->sigValueChanged().connect(
                    [this](double){ onCameraPositionSpinValueChanged(); }));
            grid->addWidget(spin, i, j * 2 + 2);
        }
    }
    vbox->addLayout(grid);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Field of View:")));
    fovSpin.setDecimals(0);
    fovSpin.setRange(1.0, 179.0);
    widgetConnections.add(
        fovSpin.sigValueChanged().connect(
            [this](double value){ onFieldOfViewSpinValueChanged(value); }));
    hbox->addWidget(&fovSpin);
    hbox->addWidget(new QLabel(_("[deg]")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Near Clip:")));
    nearClipSpin.setDecimals(3);
    nearClipSpin.setRange(0.001, 9.999);
    nearClipSpin.setSingleStep(0.001);
    widgetConnections.add(
        nearClipSpin.sigValueChanged().connect(
            [this](double){ onClipDistanceSpinValueChanged(); }));
    hbox->addWidget(&nearClipSpin);

    hbox->addWidget(new QLabel(_("Far Clip:")));
    farClipSpin.setDecimals(1);
    farClipSpin.setRange(0.1, 999.9);
    farClipSpin.setSingleStep(1.0);
    widgetConnections.add(
        farClipSpin.sigValueChanged().connect(
            [this](double){ onClipDistanceSpinValueChanged(); }));
    hbox->addWidget(&farClipSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    parallelTrackingCheck.setText(_("Parallel tracking"));
    widgetConnections.add(
        parallelTrackingCheck.sigToggled().connect(
            [this](bool on){ onParallelTrackingModeToggled(on); }));
    vbox->addWidget(&parallelTrackingCheck);
    
    interactiveViewpointChangeCheck.setText(_("Interactive viewpoint change"));
    widgetConnections.add(
        interactiveViewpointChangeCheck.sigToggled().connect(
            [this](bool on){ onInteractiveViewpointChangeToggled(on); }));
    vbox->addWidget(&interactiveViewpointChangeCheck);

    activationInSceneViewCheck.setText(_("Activate in the scene view"));
    activationCheckConnection =
        activationInSceneViewCheck.sigToggled().connect(
            [this](bool on){ onSceneViewActivationChecked(on); });
    vbox->addWidget(&activationInSceneViewCheck);

    auto buttonBox = new QDialogButtonBox(self);
    auto okButton = new PushButton(_("&OK"));
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox, &QDialogButtonBox::accepted, [this](){ this->self->accept(); });
    vbox->addWidget(buttonBox);

    self->setWindowPositionKeepingMode(true);
}

    
BodySyncCameraConfigDialog::~BodySyncCameraConfigDialog()
{
    delete impl;
}


BodySyncCameraItem* BodySyncCameraConfigDialog::showToCreateCameraItem(BodyItem* bodyItem, Link* link)
{
    setWindowTitle(_("Camera Creation"));
    
    BodySyncCameraItemPtr cameraItem = new BodySyncCameraItem;
    cameraItem->setName(format(_("{0} Camera"), bodyItem->name()));
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

    impl->showToConfigureCameraItem(cameraItem);

    return cameraItem;
}


void BodySyncCameraConfigDialog::showToConfigureCameraItem(BodySyncCameraItem* cameraItem)
{
    setWindowTitle(_("Camera Configuration"));
    impl->showToConfigureCameraItem(cameraItem);
}


void BodySyncCameraConfigDialog::Impl::showToConfigureCameraItem(BodySyncCameraItem* cameraItem)
{
    setCameraItem(cameraItem);
    self->show();
}


void BodySyncCameraConfigDialog::Impl::setCameraItem(BodySyncCameraItem* cameraItem)
{
    cameraConnection.disconnect();
    this->cameraItem = cameraItem;

    activationCheckConnection.block();

    if(!cameraItem){
        activationInSceneViewCheck.setChecked(false);

    } else {
        auto currentCamera = SceneView::instance()->sceneWidget()->renderer()->currentCamera();
        activationInSceneViewCheck.setChecked(cameraItem->perspectiveCamera() == currentCamera);

        cameraConnection =
            cameraItem->cameraTransform()->sigUpdated().connect(
                [this](const SgUpdate&){ updateWidgetsWithCurrentCameraStates(); });
        
        updateWidgetsWithCurrentCameraStates();
    }

    activationCheckConnection.unblock();
}


void BodySyncCameraConfigDialog::Impl::updateWidgetsWithCurrentCameraStates()
{
    widgetConnections.block();

    nameEdit.setText(cameraItem->name().c_str());

    Isometry3 T;
    if(globalRadio.isChecked()){
        T = cameraItem->cameraTransform()->T();
    } else {
        T = cameraItem->relativeCameraPosition();
    }
    setVectorElementSpins(T.translation(), positionSpins);
    setVectorElementSpins(Vector3(SgCamera::direction(T)), directionSpins);
    setVectorElementSpins(Vector3(SgCamera::up(T)), upSpins);
    
    auto camera = cameraItem->perspectiveCamera();
    nearClipSpin.setValue(camera->nearClipDistance());
    farClipSpin.setValue(camera->farClipDistance());
    fovSpin.setValue(degree(camera->fieldOfView()));

    parallelTrackingCheck.setChecked(cameraItem->isParallelTrackingMode());

    interactiveViewpointChangeCheck.setChecked(
        cameraItem->isInteractiveViewpointChangeEnabled());

    widgetConnections.unblock();
}


void BodySyncCameraConfigDialog::Impl::setVectorElementSpins(const Vector3& v, LengthSpinBox spins[])
{
    for(int i=0; i < 3; ++i){
        spins[i].setMeterValue(v[i]);
    }
}


void BodySyncCameraConfigDialog::Impl::setVectorElementSpins(const Vector3& v, DoubleSpinBox spins[])
{
    for(int i=0; i < 3; ++i){
        spins[i].setValue(v[i]);
    }
}


void BodySyncCameraConfigDialog::Impl::onNameEditingFinished(const std::string& name)
{
    cameraItem->setName(name);
}


void BodySyncCameraConfigDialog::Impl::alignWithBuiltinCamera()
{
    auto sceneWidget = SceneView::instance()->sceneWidget();

    auto builtin = sceneWidget->builtinPerspectiveCamera();
    auto camera = cameraItem->perspectiveCamera();
    camera->setNearClipDistance(builtin->nearClipDistance());
    camera->setFarClipDistance(builtin->farClipDistance());
    camera->setFieldOfView(builtin->fieldOfView());
    
    auto transform = cameraItem->cameraTransform();
    transform->setPosition(sceneWidget->builtinCameraTransform()->position());

    camera->notifyUpdate();
    globalRadio.setChecked(true);
}


void BodySyncCameraConfigDialog::Impl::alignWithTargetOrigin()
{
    auto transform = cameraItem->cameraTransform();
    transform->setPosition(cameraItem->targetLinkPosition());
    transform->notifyUpdate();
    localRadio.setChecked(true);
}


void BodySyncCameraConfigDialog::Impl::onCameraPositionSpinValueChanged()
{
    cameraConnection.block();
    
    Vector3 eye(positionSpins[0].meterValue(), positionSpins[1].meterValue(), positionSpins[2].meterValue());
    Vector3 dir(directionSpins[0].value(), directionSpins[1].value(), directionSpins[2].value());
    Vector3 up(upSpins[0].value(), upSpins[1].value(), upSpins[2].value());

    if(dir.norm() > 0.0 && up.norm() > 0.0){
        Isometry3 T = SgCamera::positionLookingFor(eye, dir, up);
        if(localRadio.isChecked()){
            T = cameraItem->targetLinkPosition() * T;
        }
        auto transform = cameraItem->cameraTransform();
        transform->setPosition(T);
        transform->notifyUpdate();
    }

    cameraConnection.unblock();

    cameraItem->notifyUpdate();
}


void BodySyncCameraConfigDialog::Impl::onFieldOfViewSpinValueChanged(double fov)
{
    auto camera = cameraItem->perspectiveCamera();
    camera->setFieldOfView(radian(fov));
    camera->notifyUpdate();
    cameraItem->notifyUpdate();
}


void BodySyncCameraConfigDialog::Impl::onClipDistanceSpinValueChanged()
{
    auto camera = cameraItem->perspectiveCamera();
    camera->setNearClipDistance(nearClipSpin.value());
    camera->setFarClipDistance(farClipSpin.value());
    camera->notifyUpdate();
    cameraItem->notifyUpdate();
}


void BodySyncCameraConfigDialog::Impl::onParallelTrackingModeToggled(bool on)
{
    cameraItem->setParallelTrackingMode(on);
    cameraItem->notifyUpdate();
}


void BodySyncCameraConfigDialog::Impl::onInteractiveViewpointChangeToggled(bool on)
{
    cameraItem->setInteractiveViewpointChangeEnabled(on);
    cameraItem->notifyUpdate();
}


void BodySyncCameraConfigDialog::Impl::onSceneViewActivationChecked(bool on)
{
    auto renderer = SceneView::instance()->sceneWidget()->renderer();

    if(on && !cameraItem->isChecked()){
        cameraItem->setChecked(true);
        renderer->extractPreprocessedNodes();
    }
    
    auto camera = cameraItem->perspectiveCamera();
    auto currentCamera = renderer->currentCamera();

    if(on){
        if(camera != currentCamera){
            renderer->setCurrentCamera(camera);
        }
    } else {
        if(camera == currentCamera){
            renderer->setCurrentCamera(0);
        }
    }
}


void BodySyncCameraConfigDialog::hideEvent(QHideEvent* event)
{
    impl->setCameraItem(nullptr);
    Dialog::hideEvent(event);
}
