/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBar.h"
#include "ExtensionManager.h"
#include "SceneWidget.h"
#include "GLSceneRenderer.h"
#include "ComboBox.h"
#include <cnoid/ConnectionSet>
#include <boost/bind.hpp>
#include "gettext.h"

using namespace std;
using namespace boost;
using namespace cnoid;

namespace {
SceneBar* sceneBar;

}

namespace cnoid {

class SceneBarImpl
{
public:
    SceneBar* self;

    SceneWidget* targetSceneWidget;
    SceneRenderer* targetRenderer;

    struct SceneWidgetInfo
    {
        signals::connection connectionToSigFocusChanged;
        signals::connection connectionToSigAboutToBeDestroyed;
        ~SceneWidgetInfo(){
            connectionToSigAboutToBeDestroyed.disconnect();
            connectionToSigAboutToBeDestroyed.disconnect();
        }
    };
    typedef map<SceneWidget*, SceneWidgetInfo> InfoMap;
    InfoMap sceneWidgetInfos;

    ConnectionSet connectionsToTargetSceneWidget;
        
    ToolButton* editModeToggle;
    ToolButton* firstPersonModeToggle;
    ComboBox* cameraCombo;
    ToolButton* collisionLineToggle;
    ToolButton* wireframeToggle;

    SceneBarImpl(SceneBar* self);
    void onSceneWidgetCreated(SceneWidget* sceneWidget);
    void onSceneWidgetFocusChanged(SceneWidget* sceneWidget, bool isFocused);
    void onSceneWidgetAboutToBeDestroyed(SceneWidget* sceneWidget);
    void setTargetSceneWidget(SceneWidget* sceneWidget);
    void onEditModeButtonToggled(bool on);
    void updateEditModeButton();
    void onFirstPersonModeButtonToggled(bool on);
    void updateFirstPersonModeButton();
    void onCameraComboCurrentIndexChanged(int index);
    void updateCameraCombo();
    void onCurrentCameraChanged();
    void onCollisionLineButtonToggled(bool on);
    void onWireframeButtonToggled(bool on);
};
}


SceneBar* SceneBar::instance()
{
    assert(sceneBar);
    return sceneBar;
}


void SceneBar::initialize(ExtensionManager* ext)
{
    if(!sceneBar){
        sceneBar = new SceneBar();
        ext->addToolBar(sceneBar);
    }
}


SceneBar::SceneBar()
    : ToolBar(N_("SceneBar"))
{
    impl = new SceneBarImpl(this);
}


SceneBarImpl::SceneBarImpl(SceneBar* self)
    : self(self)
{
    self->setEnabled(false);
    targetSceneWidget = 0;
    targetRenderer = 0;
    
    editModeToggle = self->addToggleButton(
        QIcon(":/Base/icons/sceneedit.png"), _("Switch to the edit mode"));
    editModeToggle->sigToggled().connect(
        bind(&SceneBarImpl::onEditModeButtonToggled, this, _1));

    firstPersonModeToggle = self->addToggleButton(
        QIcon(":/Base/icons/walkthrough.png"), _("First-person viewpoint control mode"));
    firstPersonModeToggle->sigToggled().connect(
        bind(&SceneBarImpl::onFirstPersonModeButtonToggled, this, _1));

    cameraCombo = new ComboBox();
    cameraCombo->setToolTip(_("Select a camera"));
    cameraCombo->setMinimumContentsLength(1);
    cameraCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    cameraCombo->sigCurrentIndexChanged().connect(
        bind(&SceneBarImpl::onCameraComboCurrentIndexChanged, this, _1));
    self->addWidget(cameraCombo);

    self->addButton(QIcon(":/Base/icons/viewfitting.png"), _("Move the camera to look at the objects"))
        ->sigClicked().connect(bind(&SceneWidget::viewAll, ref(targetSceneWidget)));

    collisionLineToggle = self->addToggleButton(
        QIcon(":/Base/icons/collisionlines.png"), _("Toggle the collision line visibility"));
    collisionLineToggle->sigToggled().connect(
        bind(&SceneBarImpl::onCollisionLineButtonToggled, this, _1));
    
    wireframeToggle = self->addToggleButton(
        QIcon(":/Base/icons/wireframe.png"), _("Toggle the wireframe mode"));
    wireframeToggle->sigToggled().connect(
        bind(&SceneBarImpl::onWireframeButtonToggled, this, _1));

    self->addButton(QIcon(":/Base/icons/setup.png"), _("Open the dialog to setup scene rendering"))
        ->sigClicked().connect(bind(&SceneWidget::showSetupDialog, ref(targetSceneWidget)));

    SceneWidget::sigSceneWidgetCreated().connect(bind(&SceneBarImpl::onSceneWidgetCreated, this, _1));
}


void SceneBarImpl::onSceneWidgetCreated(SceneWidget* sceneWidget)
{
    SceneWidgetInfo& info = sceneWidgetInfos[sceneWidget];

    info.connectionToSigFocusChanged =
        sceneWidget->sigWidgetFocusChanged().connect(
            bind(&SceneBarImpl::onSceneWidgetFocusChanged, this, sceneWidget, _1));

    info.connectionToSigAboutToBeDestroyed =
        sceneWidget->sigAboutToBeDestroyed().connect(
            bind(&SceneBarImpl::onSceneWidgetAboutToBeDestroyed, this, sceneWidget));

    if(!targetSceneWidget){
        setTargetSceneWidget(sceneWidget);
    }
}


void SceneBarImpl::onSceneWidgetFocusChanged(SceneWidget* sceneWidget, bool isFocused)
{
    if(isFocused){
        if(sceneWidget != targetSceneWidget){
            setTargetSceneWidget(sceneWidget);
        }
    }
}


void SceneBarImpl::onSceneWidgetAboutToBeDestroyed(SceneWidget* sceneWidget)
{
    if(sceneWidget == targetSceneWidget){
        setTargetSceneWidget(0);
    }
    sceneWidgetInfos.erase(sceneWidget);
}


SceneWidget* SceneBar::targetSceneWidget()
{
    return impl->targetSceneWidget;
}


void SceneBarImpl::setTargetSceneWidget(SceneWidget* sceneWidget)
{
    connectionsToTargetSceneWidget.disconnect();
    
    targetSceneWidget = sceneWidget;

    if(!sceneWidget){
        targetRenderer = 0;
        self->setEnabled(false);

    } else {
        targetRenderer = &sceneWidget->renderer();
        
        updateEditModeButton();
        connectionsToTargetSceneWidget.add(
            sceneWidget->sigEditModeToggled().connect(
                bind(&SceneBarImpl::updateEditModeButton, this)));

        updateFirstPersonModeButton();
        connectionsToTargetSceneWidget.add(
            sceneWidget->sigViewpointControlModeChanged().connect(
                bind(&SceneBarImpl::updateFirstPersonModeButton, this)));

        collisionLineToggle->blockSignals(true);
        collisionLineToggle->setChecked(sceneWidget->collisionLinesVisible());
        collisionLineToggle->blockSignals(false);

        wireframeToggle->blockSignals(true);
        wireframeToggle->setChecked(sceneWidget->polygonMode() != SceneWidget::FILL_MODE);
        wireframeToggle->blockSignals(false);

        updateCameraCombo();
        connectionsToTargetSceneWidget.add(
            targetRenderer->sigCamerasChanged().connect(
                bind(&SceneBarImpl::updateCameraCombo, this)));
        connectionsToTargetSceneWidget.add(
            targetRenderer->sigCurrentCameraChanged().connect(
                bind(&SceneBarImpl::onCurrentCameraChanged, this)));

        self->setEnabled(true);
    }
}


void SceneBarImpl::onEditModeButtonToggled(bool on)
{
    connectionsToTargetSceneWidget.block();
    targetSceneWidget->setEditMode(on);
    connectionsToTargetSceneWidget.unblock();
}


void SceneBarImpl::updateEditModeButton()
{
    editModeToggle->blockSignals(true);
    editModeToggle->setChecked(targetSceneWidget->isEditMode());
    editModeToggle->blockSignals(false);
}


void SceneBarImpl::onFirstPersonModeButtonToggled(bool on)
{
    connectionsToTargetSceneWidget.block();
    targetSceneWidget->setViewpointControlMode(on ? SceneWidget::FIRST_PERSON_MODE : SceneWidget::THIRD_PERSON_MODE);
    connectionsToTargetSceneWidget.unblock();
}


void SceneBarImpl::updateFirstPersonModeButton()
{
    firstPersonModeToggle->blockSignals(true);
    firstPersonModeToggle->setChecked(targetSceneWidget->viewpointControlMode() != SceneWidget::THIRD_PERSON_MODE);
    firstPersonModeToggle->blockSignals(false);
}


void SceneBarImpl::onCameraComboCurrentIndexChanged(int index)
{
    connectionsToTargetSceneWidget.block();
    targetRenderer->setCurrentCamera(index);
    connectionsToTargetSceneWidget.unblock();
}
    

void SceneBarImpl::updateCameraCombo()
{
    cameraCombo->blockSignals(true);
    
    vector<string> pathStrings;
    cameraCombo->clear();
    const int n = targetRenderer->numCameras();
    for(int i=0; i < n; ++i){
        targetRenderer->getSimplifiedCameraPathStrings(i, pathStrings);
        string label;
        if(pathStrings.empty()){
            label = str(boost::format(_("Camera %1%")) % i);
        } else if(pathStrings.size() == 1){
            label = pathStrings.front();
        } else {
            label = pathStrings.front() + " - " + pathStrings.back();
        }
        cameraCombo->addItem(label.c_str());
    }
    cameraCombo->setCurrentIndex(targetRenderer->currentCameraIndex());

    cameraCombo->blockSignals(false);
}


void SceneBarImpl::onCurrentCameraChanged()
{
    cameraCombo->blockSignals(true);
    cameraCombo->setCurrentIndex(targetRenderer->currentCameraIndex());
    cameraCombo->blockSignals(false);
}


void SceneBarImpl::onCollisionLineButtonToggled(bool on)
{
    //connectionsToTargetSceneWidget.block();
    targetSceneWidget->setCollisionLinesVisible(on);
    //connectionsToTargetSceneWidget.unblock();
}


void SceneBarImpl::onWireframeButtonToggled(bool on)
{
    //connectionsToTargetSceneWidget.block();
    targetSceneWidget->setPolygonMode(on ? SceneWidget::LINE_MODE : SceneWidget::FILL_MODE);
    //connectionsToTargetSceneWidget.unblock();
}
