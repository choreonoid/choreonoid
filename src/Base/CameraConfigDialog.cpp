#include "CameraConfigDialog.h"
#include "CameraItem.h"
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

class CameraConfigDialog::Impl
{
public:
    CameraConfigDialog* self;
    CameraItemPtr cameraItem;
    LineEdit nameEdit;
    RadioButton perspectiveRadio;
    RadioButton orthographicRadio;
    LengthSpinBox positionSpins[3];
    DoubleSpinBox directionSpins[3];
    DoubleSpinBox upSpins[3];
    DoubleSpinBox nearClipSpin;
    DoubleSpinBox farClipSpin;
    DoubleSpinBox fovSpin;
    CheckBox interactiveViewpointChangeCheck;
    QVBoxLayout* alignVBox;
    QVBoxLayout* optionVBox1;
    QVBoxLayout* optionVBox2;
    ConnectionSet widgetConnections;

    CheckBox activationInSceneViewCheck;
    Connection activationCheckConnection;

    ScopedConnection cameraConnection;

    Impl(CameraConfigDialog* self);
    void showToConfigureCameraItem(CameraItem* cameraItem);
    void setCameraItem(CameraItem* cameraItem);
    void updateWidgetsWithCurrentCameraStates();
    void setVectorElementSpins(const Vector3& v, LengthSpinBox spins[]);
    void setVectorElementSpins(const Vector3& v, DoubleSpinBox spins[]);
    void onNameEditingFinished(const std::string& name);
    void alignWithBuiltinCamera();
    void onCameraTypeRadioChanged(int type);
    void onCameraPositionSpinValueChanged();
    void onFieldOfViewSpinValueChanged(double fov);
    void onClipDistanceSpinValueChanged();
    void onInteractiveViewpointChangeToggled(bool on);
};

}


CameraConfigDialog* CameraConfigDialog::instance()
{
    static CameraConfigDialog* instance_ = new CameraConfigDialog;
    return instance_;
}


CameraConfigDialog::CameraConfigDialog()
{
    impl = new Impl(this);
}


CameraConfigDialog::Impl::Impl(CameraConfigDialog* self_)
    : self(self_)
{
    auto vbox = new QVBoxLayout;
    self->setLayout(vbox);

    auto hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Name:")));
    widgetConnections.add(
        nameEdit.sigEditingFinished().connect(
            [this](){ onNameEditingFinished(nameEdit.text().toStdString()); }));
    hbox->addWidget(&nameEdit);
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    alignVBox = new QVBoxLayout;
    auto alignButton = new PushButton(_("Align with the builtin camera"));
    alignButton->sigClicked().connect(
        [this](){ alignWithBuiltinCamera(); });
    alignVBox->addWidget(alignButton);
    hbox->addStretch();
    hbox->addLayout(alignVBox);
    hbox->addStretch();
    vbox->addLayout(hbox);

    optionVBox1 = new QVBoxLayout;
    vbox->addLayout(optionVBox1);

    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Camera type:")));
    auto cameraTypeGroup = new ButtonGroup;
    perspectiveRadio.setText(_("Perspective"));
    perspectiveRadio.setChecked(true);
    cameraTypeGroup->addButton(&perspectiveRadio, CameraItem::Perspective);
    hbox->addWidget(&perspectiveRadio);
    orthographicRadio.setText(_("Orthographic"));
    cameraTypeGroup->addButton(&orthographicRadio, CameraItem::Orthographic);
    hbox->addWidget(&orthographicRadio);
    widgetConnections.add(
        cameraTypeGroup->sigButtonToggled().connect(
            [this](int type, bool on){
                if(on){
                    onCameraTypeRadioChanged(type);
                }
            }));
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    auto grid = new QGridLayout;
    grid->addWidget(new QLabel(_("Position:")), 0, 0);
    grid->addWidget(new QLabel(_("Look-at:")), 1, 0);
    grid->addWidget(new QLabel(_("Up vector:")), 2, 0);

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

    optionVBox2 = new QVBoxLayout;
    vbox->addLayout(optionVBox2);

    interactiveViewpointChangeCheck.setText(_("Interactive viewpoint change"));
    widgetConnections.add(
        interactiveViewpointChangeCheck.sigToggled().connect(
            [this](bool on){ onInteractiveViewpointChangeToggled(on); }));
    vbox->addWidget(&interactiveViewpointChangeCheck);

    activationInSceneViewCheck.setText(_("Activate in the scene view"));
    activationCheckConnection =
        activationInSceneViewCheck.sigToggled().connect(
            [this](bool on){ cameraItem->activateCameraInSceneView(SceneView::lastFocusSceneView(), on); });
    vbox->addWidget(&activationInSceneViewCheck);

    auto buttonBox = new QDialogButtonBox(self);
    auto okButton = new PushButton(_("&OK"));
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox, &QDialogButtonBox::accepted, self, &QDialog::accept);
    vbox->addWidget(buttonBox);

    self->setWindowPositionKeepingMode(true);
}

    
CameraConfigDialog::~CameraConfigDialog()
{
    delete impl;
}


CameraItem* CameraConfigDialog::showToCreateCameraItem(Item* parentItem)
{
    setWindowTitle(_("Camera Creation"));
    
    CameraItemPtr cameraItem = new CameraItem;
    cameraItem->setName(_("Camera"));
    cameraItem->setChecked(true);
    cameraItem->setInteractiveViewpointChangeEnabled(false);

    cameraItem->cameraTransform()->setPosition(
        Isometry3::Identity() * AngleAxis(PI / 2.0, Vector3::UnitZ()) * AngleAxis(PI, Vector3::UnitX()));

    parentItem->addChildItem(cameraItem);

    impl->showToConfigureCameraItem(cameraItem);

    return cameraItem;
}


void CameraConfigDialog::showToConfigureCameraItem(CameraItem* cameraItem)
{
    setWindowTitle(_("Camera Configuration"));
    impl->showToConfigureCameraItem(cameraItem);
}


void CameraConfigDialog::Impl::showToConfigureCameraItem(CameraItem* cameraItem)
{
    setCameraItem(cameraItem);
    self->show();
}


QVBoxLayout* CameraConfigDialog::alignVBox()
{
    return impl->alignVBox;
}


QVBoxLayout* CameraConfigDialog::optionVBox1()
{
    return impl->optionVBox1;
}


QVBoxLayout* CameraConfigDialog::optionVBox2()
{
    return impl->optionVBox2;
}


CameraItem* CameraConfigDialog::cameraItem()
{
    return impl->cameraItem;
}


void CameraConfigDialog::Impl::setCameraItem(CameraItem* cameraItem)
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
                [this](const SgUpdate&){ self->updateWidgetsWithCurrentCameraStates(); });
        
        self->updateWidgetsWithCurrentCameraStates();
    }

    activationCheckConnection.unblock();
}


void CameraConfigDialog::updateWidgetsWithCurrentCameraStates()
{
    impl->updateWidgetsWithCurrentCameraStates();
}


void CameraConfigDialog::Impl::updateWidgetsWithCurrentCameraStates()
{
    widgetConnections.block();

    nameEdit.setText(cameraItem->name().c_str());

    if(cameraItem->cameraType() == CameraItem::Perspective){
        perspectiveRadio.setChecked(true);
    } else {
        orthographicRadio.setChecked(true);
    }

    Isometry3 T = self->getCurrentCameraPositionToDisplay();

    setVectorElementSpins(T.translation(), positionSpins);
    setVectorElementSpins(Vector3(SgCamera::direction(T)), directionSpins);
    setVectorElementSpins(Vector3(SgCamera::up(T)), upSpins);
    
    auto camera = cameraItem->currentCamera();
    nearClipSpin.setValue(camera->nearClipDistance());
    farClipSpin.setValue(camera->farClipDistance());
    
    fovSpin.setValue(degree(cameraItem->perspectiveCamera()->fieldOfView()));

    interactiveViewpointChangeCheck.setChecked(
        cameraItem->isInteractiveViewpointChangeEnabled());

    widgetConnections.unblock();
}


Isometry3 CameraConfigDialog::getCurrentCameraPositionToDisplay()
{
    return impl->cameraItem->cameraTransform()->T();
}


void CameraConfigDialog::setCameraPositionToDisplayToCameraTransform(const Isometry3& T)
{
    auto transform = impl->cameraItem->cameraTransform();
    transform->setPosition(T);
    transform->notifyUpdate();
}


void CameraConfigDialog::Impl::setVectorElementSpins(const Vector3& v, LengthSpinBox spins[])
{
    for(int i=0; i < 3; ++i){
        spins[i].setMeterValue(v[i]);
    }
}


void CameraConfigDialog::Impl::setVectorElementSpins(const Vector3& v, DoubleSpinBox spins[])
{
    for(int i=0; i < 3; ++i){
        spins[i].setValue(v[i]);
    }
}


void CameraConfigDialog::Impl::onNameEditingFinished(const std::string& name)
{
    cameraItem->setName(name);
}


void CameraConfigDialog::Impl::alignWithBuiltinCamera()
{
    auto sceneWidget = SceneView::lastFocusSceneView()->sceneWidget();

    SgCamera* camera;
    SgCamera* builtinCamera;
    if(perspectiveRadio.isChecked()){
        auto persCamera = cameraItem->perspectiveCamera();
        auto builtinPersCamera = sceneWidget->builtinPerspectiveCamera();
        persCamera->setFieldOfView(builtinPersCamera->fieldOfView());
        camera = persCamera;
        builtinCamera = builtinPersCamera;
    } else {
        auto orthoCamera = cameraItem->orthographicCamera();
        auto builtinOrthoCamera = sceneWidget->builtinOrthographicCamera();
        orthoCamera->setHeight(builtinOrthoCamera->height());
        camera = orthoCamera;
        builtinCamera = builtinOrthoCamera;
    }
    camera->setNearClipDistance(builtinCamera->nearClipDistance());
    camera->setFarClipDistance(builtinCamera->farClipDistance());
    
    auto transform = cameraItem->cameraTransform();
    transform->setPosition(sceneWidget->builtinCameraTransform()->position());

    camera->notifyUpdate();
}


void CameraConfigDialog::Impl::onCameraTypeRadioChanged(int type)
{
    cameraItem->setCameraType(static_cast<CameraItem::CameraType>(type));
    updateWidgetsWithCurrentCameraStates();
}


void CameraConfigDialog::Impl::onCameraPositionSpinValueChanged()
{
    cameraConnection.block();
    
    Vector3 eye(positionSpins[0].meterValue(), positionSpins[1].meterValue(), positionSpins[2].meterValue());
    Vector3 dir(directionSpins[0].value(), directionSpins[1].value(), directionSpins[2].value());
    Vector3 up(upSpins[0].value(), upSpins[1].value(), upSpins[2].value());

    if(dir.norm() > 0.0 && up.norm() > 0.0){
        Isometry3 T = SgCamera::positionLookingFor(eye, dir, up);
        self->setCameraPositionToDisplayToCameraTransform(T);
    }

    cameraConnection.unblock();

    cameraItem->notifyUpdate();
}


void CameraConfigDialog::Impl::onFieldOfViewSpinValueChanged(double fov)
{
    auto camera = cameraItem->perspectiveCamera();
    camera->setFieldOfView(radian(fov));
    camera->notifyUpdate();
    cameraItem->notifyUpdate();
}


void CameraConfigDialog::Impl::onClipDistanceSpinValueChanged()
{
    auto persCamera = cameraItem->perspectiveCamera();
    persCamera->setNearClipDistance(nearClipSpin.value());
    persCamera->setFarClipDistance(farClipSpin.value());
    persCamera->notifyUpdate();

    auto orthoCamera = cameraItem->orthographicCamera();
    orthoCamera->setNearClipDistance(nearClipSpin.value());
    orthoCamera->setFarClipDistance(farClipSpin.value());
    orthoCamera->notifyUpdate();
    
    cameraItem->notifyUpdate();
}


void CameraConfigDialog::Impl::onInteractiveViewpointChangeToggled(bool on)
{
    cameraItem->setInteractiveViewpointChangeEnabled(on);
    cameraItem->notifyUpdate();
}


void CameraConfigDialog::hideEvent(QHideEvent* event)
{
    impl->setCameraItem(nullptr);
    Dialog::hideEvent(event);
}
