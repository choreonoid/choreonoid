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
        Connection connectionToSigFocusChanged;
        Connection connectionToSigAboutToBeDestroyed;
        ~SceneWidgetInfo(){
            connectionToSigAboutToBeDestroyed.disconnect();
            connectionToSigAboutToBeDestroyed.disconnect();
        }
    };
    typedef map<SceneWidget*, SceneWidgetInfo> InfoMap;
    InfoMap sceneWidgetInfos;

    Connection sceneWidgetStateConnection;
    ConnectionSet rendererStateConnections;
        
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
    void onSceneWidgetStateChanged();
    void onEditModeButtonToggled(bool on);
    void onFirstPersonModeButtonToggled(bool on);
    void onSceneRendererCamerasChanged();
    void onSceneRendererCurrentCameraChanged();
    void onCameraComboCurrentIndexChanged(int index);
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
    self->setVisibleByDefault(true);
    
    self->setEnabled(false);
    targetSceneWidget = 0;
    targetRenderer = 0;
    
    editModeToggle = self->addToggleButton(
        QIcon(":/Base/icons/sceneedit.png"), _("Switch to the edit mode"));
    editModeToggle->sigToggled().connect(
        boost::bind(&SceneBarImpl::onEditModeButtonToggled, this, _1));

    firstPersonModeToggle = self->addToggleButton(
        QIcon(":/Base/icons/walkthrough.png"), _("First-person viewpoint control mode"));
    firstPersonModeToggle->sigToggled().connect(
        boost::bind(&SceneBarImpl::onFirstPersonModeButtonToggled, this, _1));

    cameraCombo = new ComboBox();
    cameraCombo->setToolTip(_("Select a camera"));
    cameraCombo->setMinimumContentsLength(1);
    cameraCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    cameraCombo->sigCurrentIndexChanged().connect(
        boost::bind(&SceneBarImpl::onCameraComboCurrentIndexChanged, this, _1));
    self->addWidget(cameraCombo);

    self->addButton(QIcon(":/Base/icons/viewfitting.png"), _("Move the camera to look at the objects"))
        ->sigClicked().connect(boost::bind(&SceneWidget::viewAll, boost::ref(targetSceneWidget)));

    collisionLineToggle = self->addToggleButton(
        QIcon(":/Base/icons/collisionlines.png"), _("Toggle the collision line visibility"));
    collisionLineToggle->sigToggled().connect(
        boost::bind(&SceneBarImpl::onCollisionLineButtonToggled, this, _1));
    
    wireframeToggle = self->addToggleButton(
        QIcon(":/Base/icons/wireframe.png"), _("Toggle the wireframe mode"));
    wireframeToggle->sigToggled().connect(
        boost::bind(&SceneBarImpl::onWireframeButtonToggled, this, _1));

    self->addButton(QIcon(":/Base/icons/setup.png"), _("Open the dialog to setup scene rendering"))
        ->sigClicked().connect(boost::bind(&SceneWidget::showSetupDialog, boost::ref(targetSceneWidget)));

    SceneWidget::sigSceneWidgetCreated().connect(boost::bind(&SceneBarImpl::onSceneWidgetCreated, this, _1));
}


void SceneBarImpl::onSceneWidgetCreated(SceneWidget* sceneWidget)
{
    SceneWidgetInfo& info = sceneWidgetInfos[sceneWidget];

    info.connectionToSigFocusChanged =
        sceneWidget->sigWidgetFocusChanged().connect(
            boost::bind(&SceneBarImpl::onSceneWidgetFocusChanged, this, sceneWidget, _1));

    info.connectionToSigAboutToBeDestroyed =
        sceneWidget->sigAboutToBeDestroyed().connect(
            boost::bind(&SceneBarImpl::onSceneWidgetAboutToBeDestroyed, this, sceneWidget));

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
    sceneWidgetStateConnection.disconnect();
    rendererStateConnections.disconnect();
    
    targetSceneWidget = sceneWidget;

    if(!sceneWidget){
        targetRenderer = 0;
        self->setEnabled(false);

    } else {
        targetRenderer = &sceneWidget->renderer();

        onSceneWidgetStateChanged();

        sceneWidgetStateConnection =
            sceneWidget->sigStateChanged().connect(
                boost::bind(&SceneBarImpl::onSceneWidgetStateChanged, this));

        onSceneRendererCamerasChanged();
        
        rendererStateConnections.add(
            targetRenderer->sigCamerasChanged().connect(
                boost::bind(&SceneBarImpl::onSceneRendererCamerasChanged, this)));
        
        rendererStateConnections.add(
            targetRenderer->sigCurrentCameraChanged().connect(
                boost::bind(&SceneBarImpl::onSceneRendererCurrentCameraChanged, this)));

        self->setEnabled(true);
    }
}


void SceneBarImpl::onSceneWidgetStateChanged()
{
    editModeToggle->blockSignals(true);
    editModeToggle->setChecked(targetSceneWidget->isEditMode());
    editModeToggle->blockSignals(false);

    firstPersonModeToggle->blockSignals(true);
    firstPersonModeToggle->setChecked(targetSceneWidget->viewpointControlMode() != SceneWidget::THIRD_PERSON_MODE);
    firstPersonModeToggle->blockSignals(false);

    collisionLineToggle->blockSignals(true);
    collisionLineToggle->setChecked(targetSceneWidget->collisionLinesVisible());
    collisionLineToggle->blockSignals(false);
    
    wireframeToggle->blockSignals(true);
    wireframeToggle->setChecked(targetSceneWidget->polygonMode() != SceneWidget::FILL_MODE);
    wireframeToggle->blockSignals(false);
}


void SceneBarImpl::onEditModeButtonToggled(bool on)
{
    sceneWidgetStateConnection.block();
    targetSceneWidget->setEditMode(on);
    sceneWidgetStateConnection.unblock();
}


void SceneBarImpl::onFirstPersonModeButtonToggled(bool on)
{
    sceneWidgetStateConnection.block();
    targetSceneWidget->setViewpointControlMode(on ? SceneWidget::FIRST_PERSON_MODE : SceneWidget::THIRD_PERSON_MODE);
    sceneWidgetStateConnection.unblock();
}


void SceneBarImpl::onCollisionLineButtonToggled(bool on)
{
    sceneWidgetStateConnection.block();
    targetSceneWidget->setCollisionLinesVisible(on);
    sceneWidgetStateConnection.unblock();
}


void SceneBarImpl::onWireframeButtonToggled(bool on)
{
    sceneWidgetStateConnection.block();
    targetSceneWidget->setPolygonMode(on ? SceneWidget::LINE_MODE : SceneWidget::FILL_MODE);
    sceneWidgetStateConnection.unblock();
}


void SceneBarImpl::onSceneRendererCamerasChanged()
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


void SceneBarImpl::onSceneRendererCurrentCameraChanged()
{
    cameraCombo->blockSignals(true);
    cameraCombo->setCurrentIndex(targetRenderer->currentCameraIndex());
    cameraCombo->blockSignals(false);
}


void SceneBarImpl::onCameraComboCurrentIndexChanged(int index)
{
    rendererStateConnections.block();
    targetRenderer->setCurrentCamera(index);
    rendererStateConnections.unblock();
}
