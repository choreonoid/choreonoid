#include "SceneBar.h"
#include "SceneView.h"
#include "SceneViewConfig.h"
#include "SceneWidget.h"
#include "ExtensionManager.h"
#include "InteractiveCameraTransform.h"
#include "Archive.h"
#include "ComboBox.h"
#include "ButtonGroup.h"
#include <cnoid/ConnectionSet>
#include <cnoid/SceneRenderer>
#include <cnoid/SceneCameras>
#include <cnoid/Format>
#include <cnoid/EigenUtil>
#include "gettext.h"

using namespace std;
using namespace cnoid;

namespace {

SceneBar* sceneBar;

enum ElementId {
    EditModeToggle = 0,
    FirstPersonModeToggle = 1,
    CameraCombo = 2,
    FittingButton = 3,
    VertexToggle = 4,
    WireframeToggle = 5,
    SolidWireframeToggle = 6,
    SolidPolygonToggle = 7,
    HighlightToggle = 8,
    VisualModelToggle = 9,
    ModelTypeFlipButton = 10,
    CollisionModelToggle = 11,
    CollisionLineToggle = 12,
    ConfigButton = 13,

    FrontViewButton = 70,
    BackViewButton = 71,
    TopViewButton = 72,
    BottomViewButton = 73,
    RightViewButton = 74,
    LeftViewButton = 75,
    IsometricViewButton = 76
};

}

namespace cnoid {

class SceneBar::Impl
{
public:
    SceneBar* self;

    SceneView* currentSceneView;
    ScopedConnection sceneViewFocusConnection;
    ScopedConnectionSet sceneViewConnections;
    
    ToolButton* editModeToggle;
    ToolButton* firstPersonModeToggle;
    ComboBox* cameraCombo;
    ToolButton* vertexToggle;
    ButtonGroup polygonModeGroup;
    ToolButton* visualModelToggle;
    ToolButton* highlightToggle;
    ToolButton* modelTypeFlipButton;
    ToolButton* collisionModelToggle;
    ToolButton* collisionLineToggle;

    bool isViewButtonSetEnabled;

    struct CustomModeButtonInfo {
        ToolButton* button;
        int modeId;
    };
    vector<CustomModeButtonInfo> customModeButtons;
    ButtonGroup customModeButtonGroup;

    Impl(SceneBar* self);
    void initialize();
    void onCustomModeButtonToggled(int mode, bool on);
    void onCurrentSceneViewDeactivated();
    void setCurrentSceneView(SceneView* sceneView);
    void onSceneWidgetStateChanged(SceneWidget* sceneWidget);
    void onSceneRendererCamerasChanged(SceneWidget* sceneWidget);
    void onSceneRendererCurrentCameraChanged(SceneWidget* sceneWidget);
    void onCameraComboCurrentIndexChanged(int index);
    void onEditModeButtonToggled(bool on);
    void onFirstPersonModeButtonToggled(bool on);
    void onPolygonModeButtonToggled();
    void onHighlightingToggled(bool on);
    void flipVisibleModels();
    void updateCollisionModelVisibility();
    void onCollisionLineButtonToggled(bool on);
    void enableViewButtonSet();
    void onViewButtonClicked(ElementId button);
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
        sceneBar = new SceneBar;
        ext->addToolBar(sceneBar);
    }
}


SceneBar::SceneBar()
    : ToolBar(N_("SceneBar"))
{
    impl = new Impl(this);
    impl->initialize();
}


SceneBar::Impl::Impl(SceneBar* self)
    : self(self)
{

}


void SceneBar::Impl::initialize()
{
    self->setVisibleByDefault(true);
    self->setEnabled(false);

    currentSceneView = nullptr;
    
    editModeToggle = self->addToggleButton(":/Base/icon/sceneedit.svg", EditModeToggle);
    editModeToggle->setToolTip(_("Switch to the edit mode"));
    editModeToggle->sigToggled().connect([this](bool on){ onEditModeButtonToggled(on); });

    customModeButtonGroup.setExclusive(false);
    customModeButtonGroup.sigButtonToggled().connect(
        [this](int mode, bool on){ onCustomModeButtonToggled(mode, on); });
    
    firstPersonModeToggle = self->addToggleButton(":/Base/icon/walkthrough.svg", FirstPersonModeToggle);
    firstPersonModeToggle->setToolTip(_("First-person viewpoint control mode"));
    firstPersonModeToggle->sigToggled().connect([this](bool on){ onFirstPersonModeButtonToggled(on); });

    cameraCombo = new ComboBox;

    /**
       It may be better to adjust the combo box size depending on the lengths of the available camera names
       every time the avaiable camera set changes. However, it seems impossible to achive that using the
       default implementation of ComboBox because its minimum size hint does not seem to change after the
       size is determined at the first time the combo box is shown. Even if the size adjust policy is
       changed or the adjustSize function is executed for the combo box and the tool bar, the combo box
       size will return to its first determined size when the tool bar layout is processed by ToolBarArea,
       which uses the minimum size hint to determine the width of each tool bar. Therefore the constant
       minimum contents length is used in the current implementation.
    */
    cameraCombo->setMinimumContentsLength(10);
    cameraCombo->setSizeAdjustPolicy(QComboBox::AdjustToContents);
    
    cameraCombo->setToolTip(_("Projection method / camera selection"));
    cameraCombo->sigCurrentIndexChanged().connect(
        [this](int index){ onCameraComboCurrentIndexChanged(index); });
    self->addWidget(cameraCombo, CameraCombo);

    auto fittingButton = self->addButton(":/Base/icon/viewfitting.svg", FittingButton);
    fittingButton->setToolTip(_("Move the camera to look at the objects"));
    fittingButton->sigClicked().connect(
        [this]{
            auto sceneWidget = currentSceneView->sceneWidget();
            sceneWidget->viewAll();
            sceneWidget->setViewpointOperationMode(SceneWidget::ThirdPersonMode);
        });

    vertexToggle = self->addToggleButton(":/Base/icon/vertex.svg", VertexToggle);
    vertexToggle->setToolTip(_("Vertex rendering"));
    vertexToggle->sigToggled().connect([this](bool){ onPolygonModeButtonToggled(); });

    auto wireframeToggle = self->addToggleButton(":/Base/icon/wireframe.svg", WireframeToggle);
    wireframeToggle->setToolTip(_("Wireframe rendering"));
    polygonModeGroup.addButton(wireframeToggle, 0);

    auto solidWireframeToggle =
        self->addToggleButton(":/Base/icon/solidwireframe.svg", SolidWireframeToggle);
    solidWireframeToggle->setToolTip(_("Solid wireframe rendering"));
    polygonModeGroup.addButton(solidWireframeToggle, 1);

    auto solidPolygonToggle = self->addToggleButton(":/Base/icon/solidpolygon.svg", SolidPolygonToggle);
    solidPolygonToggle->setToolTip(_("Polygon rendering"));
    polygonModeGroup.addButton(solidPolygonToggle, 2);

    polygonModeGroup.sigButtonToggled().connect(
        [this](int, bool on){
            if(on){ onPolygonModeButtonToggled(); }
        });

    highlightToggle = self->addToggleButton(":/Base/icon/highlight.svg", HighlightToggle);
    highlightToggle->setToolTip(_("Highlight selected objects"));
    highlightToggle->sigToggled().connect([this](bool on){ onHighlightingToggled(on); });

    visualModelToggle = self->addToggleButton(":/Base/icon/visualshape.svg", VisualModelToggle);
    visualModelToggle->setToolTip(_("Show visual models"));
    visualModelToggle->setChecked(true);
    visualModelToggle->sigToggled().connect([this](bool){ updateCollisionModelVisibility(); });

    modelTypeFlipButton = self->addButton(":/Base/icon/shapeflip.svg", ModelTypeFlipButton);
    modelTypeFlipButton->setToolTip(_("Flip active model types"));
    modelTypeFlipButton->sigClicked().connect([this](){ flipVisibleModels(); });

    collisionModelToggle = self->addToggleButton(":/Base/icon/collisionshape.svg", CollisionModelToggle);
    collisionModelToggle->setToolTip(_("Show the collision detection models"));
    collisionModelToggle->sigToggled().connect([this](bool){ updateCollisionModelVisibility();});

    collisionLineToggle = self->addToggleButton(":/Base/icon/collisionlines.svg", CollisionLineToggle);
    collisionLineToggle->setToolTip(_("Toggle the collision line visibility"));
    collisionLineToggle->sigToggled().connect([this](bool on){ onCollisionLineButtonToggled(on); });

    auto configButton = self->addButton(":/Base/icon/setup.svg", ConfigButton);
    configButton->setToolTip(_("Show the config dialog"));
    configButton->sigClicked().connect([this]{ currentSceneView->sceneViewConfig()->showConfigDialog(); });

    sceneViewFocusConnection =
        SceneView::sigLastFocusSceneViewChanged().connect(
            [this](SceneView* view){ setCurrentSceneView(view); });

    setCurrentSceneView(SceneView::instance());

    isViewButtonSetEnabled = false;
}


SceneBar::~SceneBar()
{
    delete impl;
}


void SceneBar::addCustomModeButton(int modeId, const QIcon& icon, const QString& caption)
{
    int position = elementPosition(EditModeToggle) + 1 + impl->customModeButtons.size();
    auto button = setInsertionPosition(position).addRadioButton(icon);
    button->setToolTip(caption);
    impl->customModeButtonGroup.addButton(button, modeId);
    impl->customModeButtons.push_back({ button, modeId });
}


void SceneBar::removeCustomModeButton(int modeId)
{
    auto p = impl->customModeButtons.begin();
    while(p != impl->customModeButtons.end()){
        if(p->modeId == modeId){
            delete p->button;
            impl->customModeButtons.erase(p);
            break;
        }
        ++p;
    }
}


void SceneBar::Impl::onCustomModeButtonToggled(int mode, bool on)
{
    if(on){
        customModeButtonGroup.blockSignals(true);
        for(auto& button : customModeButtonGroup.buttons()){
            int id = customModeButtonGroup.id(button);
            if(id != mode && button->isChecked()){
                button->setChecked(false);
            }
        }
        customModeButtonGroup.blockSignals(false);
    } else {
        mode = 0;
    }
    currentSceneView->setCustomMode(mode);
}


SceneWidget* SceneBar::targetSceneWidget()
{
    return impl->currentSceneView->sceneWidget();
}


void SceneBar::Impl::setCurrentSceneView(SceneView* sceneView)
{
    sceneViewConnections.disconnect();

    currentSceneView = sceneView;

    if(!sceneView){
        self->setEnabled(false);

    } else {
        auto sceneWidget = sceneView->sceneWidget();
        auto renderer = sceneWidget->renderer();

        onSceneWidgetStateChanged(sceneWidget);
        onSceneRendererCamerasChanged(sceneWidget);

        sceneViewConnections.add(
            sceneView->sigDeactivated().connect(
                [this](){ onCurrentSceneViewDeactivated(); }));
        
        sceneViewConnections.add(
            sceneWidget->sigStateChanged().connect(
                [this, sceneWidget](){ onSceneWidgetStateChanged(sceneWidget); }));

        sceneViewConnections.add(
            renderer->sigCamerasChanged().connect(
                [this, sceneWidget](){ onSceneRendererCamerasChanged(sceneWidget); }));
        
        sceneViewConnections.add(
            renderer->sigCurrentCameraChanged().connect(
                [this, sceneWidget](){ onSceneRendererCurrentCameraChanged(sceneWidget); }));

        self->setEnabled(true);
    }
}


void SceneBar::Impl::onCurrentSceneViewDeactivated()
{
    setCurrentSceneView(SceneView::instance());
}


void SceneBar::Impl::onSceneWidgetStateChanged(SceneWidget* sceneWidget)
{
    editModeToggle->blockSignals(true);
    editModeToggle->setChecked(sceneWidget->isEditMode());
    editModeToggle->blockSignals(false);

    customModeButtonGroup.blockSignals(true);
    int customMode = sceneWidget->activeCustomMode();
    if(customMode >= 2){
        if(auto modeButton = customModeButtonGroup.button(customMode)){
            modeButton->setChecked(true);
        }
    } else {
        if(auto checked = customModeButtonGroup.checkedButton()){
            checked->setChecked(false);
        }
    }
    customModeButtonGroup.blockSignals(false);

    firstPersonModeToggle->blockSignals(true);
    firstPersonModeToggle->setChecked(
        sceneWidget->viewpointOperationMode() != SceneWidget::ThirdPersonMode);
    firstPersonModeToggle->blockSignals(false);

    int polygonFlags = sceneWidget->visiblePolygonElements();
    
    vertexToggle->blockSignals(true);
    vertexToggle->setChecked(polygonFlags & SceneWidget::PolygonVertex);
    vertexToggle->blockSignals(false);

    polygonModeGroup.blockSignals(true);
    if(polygonFlags & SceneWidget::PolygonFace){
        if(polygonFlags & SceneWidget::PolygonEdge){
            polygonModeGroup.button(1)->setChecked(true);
        } else {
            polygonModeGroup.button(2)->setChecked(true);
        }
    } else {
        if(polygonFlags & SceneWidget::PolygonEdge){
            polygonModeGroup.button(0)->setChecked(true);
        }
    }
    polygonModeGroup.blockSignals(false);

    highlightToggle->blockSignals(true);
    highlightToggle->setChecked(sceneWidget->isHighlightingEnabled());
    highlightToggle->blockSignals(false);
    
    collisionLineToggle->blockSignals(true);
    collisionLineToggle->setChecked(sceneWidget->collisionLineVisibility());
    collisionLineToggle->blockSignals(false);
}


void SceneBar::Impl::onSceneRendererCamerasChanged(SceneWidget* sceneWidget)
{
    cameraCombo->blockSignals(true);
    cameraCombo->clear();

    auto renderer = sceneWidget->renderer();
    SgCamera* builtinPersCamera = sceneWidget->builtinPerspectiveCamera();
    SgCamera* builtinOrthoCamera = sceneWidget->builtinOrthographicCamera();
    vector<string> pathStrings;

    const int n = renderer->numCameras();
    
    for(int i=0; i < n; ++i){
        auto camera = renderer->camera(i);
        if(camera == builtinPersCamera || camera == builtinOrthoCamera){
            QString label(dgettext(CNOID_GETTEXT_DOMAIN_NAME, camera->name().c_str()));
            cameraCombo->addItem(label);
        } else {
            renderer->getSimplifiedCameraPathStrings(i, pathStrings);
            string label;
            if(pathStrings.empty()){
                label = formatC("Camera {}", i);
            } else if(pathStrings.size() == 1){
                label = pathStrings.front();
            } else {
                label = pathStrings.back() + " - " + pathStrings.front();
            }
            cameraCombo->addItem(label.c_str());
        }
    }

    cameraCombo->setCurrentIndex(renderer->currentCameraIndex());

    cameraCombo->blockSignals(false);
}


void SceneBar::Impl::onSceneRendererCurrentCameraChanged(SceneWidget* sceneWidget)
{
    cameraCombo->blockSignals(true);
    cameraCombo->setCurrentIndex(sceneWidget->renderer()->currentCameraIndex());
    cameraCombo->blockSignals(false);
}


void SceneBar::Impl::onCameraComboCurrentIndexChanged(int index)
{
    sceneViewConnections.block();
    currentSceneView->renderer()->setCurrentCamera(index);
    sceneViewConnections.unblock();
}


void SceneBar::Impl::onEditModeButtonToggled(bool on)
{
    auto sceneWidget = currentSceneView->sceneWidget();
    sceneViewConnections.block();
    sceneWidget->setEditMode(on);
    sceneViewConnections.unblock();

    // Return the button to the standard state when the edit mode is blocked.
    if(on && !sceneWidget->isEditMode()){
        editModeToggle->blockSignals(true);
        editModeToggle->setChecked(false);
        editModeToggle->blockSignals(false);
    }
}


void SceneBar::Impl::onFirstPersonModeButtonToggled(bool on)
{
    sceneViewConnections.block();
    currentSceneView->sceneWidget()->setViewpointOperationMode(
        on ? SceneWidget::FirstPersonMode : SceneWidget::ThirdPersonMode);
    sceneViewConnections.unblock();
}


void SceneBar::Impl::onPolygonModeButtonToggled()
{
    sceneViewConnections.block();
    
    int flags = 0;
    if(vertexToggle->isChecked()){
        flags |= SceneWidget::PolygonVertex;
    }
    switch(polygonModeGroup.checkedId()){
    case 0:
        flags |= SceneWidget::PolygonEdge;
        break;
    case 1:
        flags |= (SceneWidget::PolygonEdge | SceneWidget::PolygonFace);
        break;
    case 2:
    default:
        flags |= SceneWidget::PolygonFace;
        break;
    }
    currentSceneView->sceneWidget()->setVisiblePolygonElements(flags);
    
    sceneViewConnections.unblock();
}


void SceneBar::Impl::onHighlightingToggled(bool on)
{
    sceneViewConnections.block();
    currentSceneView->sceneWidget()->setHighlightingEnabled(on);
    sceneViewConnections.unblock();
}


void SceneBar::Impl::flipVisibleModels()
{
    sceneViewConnections.block();
    visualModelToggle->toggle();
    collisionModelToggle->toggle();
    sceneViewConnections.unblock();
    updateCollisionModelVisibility();
}


void SceneBar::Impl::updateCollisionModelVisibility()
{
    int mode = 0;
    if(visualModelToggle->isChecked()){
        mode = 1;
    }
    if(collisionModelToggle->isChecked()){
        mode += 2;
    }
    auto sceneWidget = currentSceneView->sceneWidget();
    sceneWidget->renderer()->setProperty(
        SceneRenderer::PropertyKey("collisionDetectionModelVisibility"), mode);
    sceneWidget->renderScene();
}


void SceneBar::Impl::onCollisionLineButtonToggled(bool on)
{
    sceneViewConnections.block();
    currentSceneView->sceneWidget()->setCollisionLineVisibility(on);
    sceneViewConnections.unblock();
}


void SceneBar::Impl::enableViewButtonSet()
{
    if(isViewButtonSetEnabled){
        return;
    }

    ToolButton* button;

    button = self->addButton(":/Base/icon/frontview.svg", FrontViewButton);
    button->setToolTip(_("Front view"));
    button->sigClicked().connect([this]{ onViewButtonClicked(FrontViewButton); });

    button = self->addButton(":/Base/icon/backview.svg", BackViewButton);
    button->setToolTip(_("Back view"));
    button->sigClicked().connect([this]{ onViewButtonClicked(BackViewButton); });

    button = self->addButton(":/Base/icon/topview.svg", TopViewButton);
    button->setToolTip(_("Top view"));
    button->sigClicked().connect([this]{ onViewButtonClicked(TopViewButton); });

    button = self->addButton(":/Base/icon/bottomview.svg", BottomViewButton);
    button->setToolTip(_("Bottom view"));
    button->sigClicked().connect([this]{ onViewButtonClicked(BottomViewButton); });

    button = self->addButton(":/Base/icon/rightview.svg", RightViewButton);
    button->setToolTip(_("Right view"));
    button->sigClicked().connect([this]{ onViewButtonClicked(RightViewButton); });

    button = self->addButton(":/Base/icon/leftview.svg", LeftViewButton);
    button->setToolTip(_("Left view"));
    button->sigClicked().connect([this]{ onViewButtonClicked(LeftViewButton); });

    button = self->addButton(":/Base/icon/isometricview.svg", IsometricViewButton);
    button->setToolTip(_("Isometric view"));
    button->sigClicked().connect([this]{ onViewButtonClicked(IsometricViewButton); });
    
    isViewButtonSetEnabled = true;
}


void SceneBar::Impl::onViewButtonClicked(ElementId button)
{
    if(currentSceneView){
        AngleAxis aa;
        switch(button){
        case FrontViewButton:
            aa = AngleAxis(M_PI_2, Vector3::UnitZ()) * AngleAxis(M_PI_2, Vector3::UnitX());
            break;
        case BackViewButton:
            aa = AngleAxis(-M_PI_2, Vector3::UnitZ()) * AngleAxis(M_PI_2, Vector3::UnitX());
            break;
        case TopViewButton:
            aa = AngleAxis(-M_PI_2, Vector3::UnitZ());
            break;
        case BottomViewButton:
            aa = AngleAxis(M_PI_2, Vector3::UnitZ()) * AngleAxis(M_PI, Vector3::UnitX());
            break;
        case RightViewButton:
            aa = AngleAxis(M_PI, Vector3::UnitZ()) * AngleAxis(M_PI_2, Vector3::UnitX());
            break;
        case LeftViewButton:
            aa = AngleAxis(M_PI_2, Vector3::UnitX());
            break;
        case IsometricViewButton:
            aa =
                AngleAxis(M_PI_4, Vector3::UnitZ()) * AngleAxis(radian(-35.264), Vector3::UnitY()) *
                AngleAxis(M_PI_2, Vector3::UnitZ()) * AngleAxis(M_PI_2, Vector3::UnitX());
            break;
        default:
            break;
        }
        auto sceneWidget = currentSceneView->sceneWidget();
        if(auto transform = sceneWidget->activeInteractiveCameraTransform()){
            transform->setRotation(aa);
        } else {
            sceneWidget->renderer()->setCurrentCamera(sceneWidget->builtinOrthographicCamera());
            sceneWidget->builtinCameraTransform()->setRotation(aa);
        }
        sceneWidget->viewAll();
    }
}


bool SceneBar::restoreState(const Archive& archive)
{
    if(archive.get("enable_six_sided_view_buttons", false)){
        impl->enableViewButtonSet();
    }
    return ToolBar::restoreState(archive);
}
