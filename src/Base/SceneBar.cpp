/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBar.h"
#include "SceneView.h"
#include "SceneWidget.h"
#include "ExtensionManager.h"
#include "MenuManager.h"
#include "RootItem.h"
#include "RenderableItem.h"
#include "ItemList.h"
#include "MessageView.h"
#include "AppConfig.h"
#include "ComboBox.h"
#include "ButtonGroup.h"
#include <cnoid/ConnectionSet>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneRenderer>
#include <fmt/format.h>
#include "gettext.h"

using namespace std;
using namespace cnoid;
using fmt::format;

namespace {

SceneBar* sceneBar;

// "tool" menu commands
void putSceneStatistics();

}

namespace cnoid {

class SceneBar::Impl
{
public:
    SceneBar* self;

    SceneView* currentSceneView;
    SceneWidget* currentSceneWidget;
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

    struct CustomModeButtonInfo {
        ToolButton* button;
        int id;
    };
    vector<CustomModeButtonInfo> customModeButtons;
    ButtonGroup customModeButtonGroup;

    MappingPtr config;

    Impl(SceneBar* self);
    void initialize();
    void onCustomModeButtonToggled(int mode, bool on);
    void onCurrentSceneViewDeactivated();
    void setCurrentSceneView(SceneView* sceneView);
    void onSceneWidgetStateChanged();
    void onSceneRendererCamerasChanged();
    void onSceneRendererCurrentCameraChanged();
    void onCameraComboCurrentIndexChanged(int index);
    void onEditModeButtonToggled(bool on);
    void onFirstPersonModeButtonToggled(bool on);
    void onPolygonModeButtonToggled();
    void onHighlightingToggled(bool on);
    void flipVisibleModels();
    void updateCollisionModelVisibility();
    void onCollisionLineButtonToggled(bool on);
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
        ext->menuManager().setPath("/Tools").addItem(N_("Put scene statistics"))
            ->sigTriggered().connect(putSceneStatistics);
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
    currentSceneWidget = nullptr;
    
    config = AppConfig::archive()->openMapping("SceneBar");
    
    editModeToggle = self->addToggleButton(
        QIcon(":/Base/icon/sceneedit.svg"), _("Switch to the edit mode"));
    editModeToggle->sigToggled().connect(
        [&](bool on){ onEditModeButtonToggled(on); });

    customModeButtonGroup.setExclusive(false);
    customModeButtonGroup.sigButtonToggled().connect(
        [&](int mode, bool on){ onCustomModeButtonToggled(mode, on); });
    
    firstPersonModeToggle = self->addToggleButton(
        QIcon(":/Base/icon/walkthrough.svg"), _("First-person viewpoint control mode"));
    firstPersonModeToggle->sigToggled().connect(
        [&](bool on){ onFirstPersonModeButtonToggled(on); });

    cameraCombo = new ComboBox();
    cameraCombo->setToolTip(_("Select a camera"));
    cameraCombo->setMinimumContentsLength(6);
    cameraCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    cameraCombo->sigCurrentIndexChanged().connect(
        [&](int index){ onCameraComboCurrentIndexChanged(index); });
    self->addWidget(cameraCombo);

    self->addButton(QIcon(":/Base/icon/viewfitting.svg"), _("Move the camera to look at the objects"))
        ->sigClicked().connect([&](){
                currentSceneWidget->viewAll();
                currentSceneWidget->setViewpointOperationMode(SceneWidget::ThirdPersonMode);
            });

    self->addSpacing();

    vertexToggle = self->addToggleButton(
        QIcon(":/Base/icon/vertex.svg"), _("Vertex rendering"));
    vertexToggle->sigToggled().connect(
        [&](bool){ onPolygonModeButtonToggled(); });

    auto wireframeToggle = self->addToggleButton(
        QIcon(":/Base/icon/wireframe.svg"), _("Wireframe rendering"));
    polygonModeGroup.addButton(wireframeToggle, 0);

    auto solidWireframeToggle = self->addToggleButton(
        QIcon(":/Base/icon/solidwireframe.svg"), _("Solid wireframe rendering"));
        
    polygonModeGroup.addButton(solidWireframeToggle, 1);

    auto solidPolygonToggle = self->addToggleButton(
        QIcon(":/Base/icon/solidpolygon.svg"), _("Polygon rendering"));
    polygonModeGroup.addButton(solidPolygonToggle, 2);

    polygonModeGroup.sigButtonToggled().connect(
        [&](int, bool on){
            if(on){ onPolygonModeButtonToggled(); }
        });

    highlightToggle = self->addToggleButton(
        QIcon(":/Base/icon/highlight.svg"), _("Highlight selected objects"));
    highlightToggle->sigToggled().connect(
        [&](bool on){ onHighlightingToggled(on); });

    visualModelToggle = self->addToggleButton(
        QIcon(":/Base/icon/visualshape.svg"), _("Show visual models"));
    visualModelToggle->setChecked(true);
    visualModelToggle->sigToggled().connect(
        [&](bool){ updateCollisionModelVisibility(); });

    modelTypeFlipButton = self->addButton(
        QIcon(":/Base/icon/shapeflip.svg"), _("Flip active model types"));
    modelTypeFlipButton->sigClicked().connect(
        [&](){ flipVisibleModels(); });

    collisionModelToggle = self->addToggleButton(
        QIcon(":/Base/icon/collisionshape.svg"), _("Show the collision detection models"));
    collisionModelToggle->sigToggled().connect(
        [&](bool){ updateCollisionModelVisibility(); });
    
    collisionLineToggle = self->addToggleButton(
        QIcon(":/Base/icon/collisionlines.svg"), _("Toggle the collision line visibility"));
    collisionLineToggle->sigToggled().connect(
        [&](bool on){ onCollisionLineButtonToggled(on); });

    self->addButton(QIcon(":/Base/icon/setup.svg"), _("Show the config dialog"))
        ->sigClicked().connect([&](){ currentSceneWidget->showConfigDialog(); });

    sceneViewFocusConnection =
        SceneView::sigLastFocusViewChanged().connect(
            [&](SceneView* view){ setCurrentSceneView(view); });

    setCurrentSceneView(SceneView::instance());
}


SceneBar::~SceneBar()
{
    delete impl;
}


void SceneBar::addCustomModeButton(int id, const QIcon& icon, const QString& caption)
{
    int position = 1 + impl->customModeButtons.size();
    auto button = setInsertionPosition(position).addRadioButton(icon, caption);
    impl->customModeButtonGroup.addButton(button, id);
    impl->customModeButtons.push_back({ button, id });
}


void SceneBar::removeCustomModeButton(int id)
{
    auto p = impl->customModeButtons.begin();
    while(p != impl->customModeButtons.end()){
        if(p->id == id){
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
    return impl->currentSceneWidget;
}


void SceneBar::Impl::setCurrentSceneView(SceneView* sceneView)
{
    sceneViewConnections.disconnect();

    currentSceneView = sceneView;

    if(!sceneView){
        currentSceneWidget = nullptr;
        self->setEnabled(false);

    } else {
        currentSceneWidget = sceneView->sceneWidget();
        auto renderer = currentSceneWidget->renderer();

        onSceneWidgetStateChanged();
        onSceneRendererCamerasChanged();

        sceneViewConnections.add(
            sceneView->sigDeactivated().connect(
                [&](){ onCurrentSceneViewDeactivated(); }));
        
        sceneViewConnections.add(
            currentSceneWidget->sigStateChanged().connect(
                [&](){ onSceneWidgetStateChanged(); }));

        sceneViewConnections.add(
            renderer->sigCamerasChanged().connect(
                [this](){ onSceneRendererCamerasChanged(); }));
        
        sceneViewConnections.add(
            renderer->sigCurrentCameraChanged().connect(
                [this](){ onSceneRendererCurrentCameraChanged(); }));

        self->setEnabled(true);
    }
}


void SceneBar::Impl::onCurrentSceneViewDeactivated()
{
    setCurrentSceneView(SceneView::instance());
}


void SceneBar::Impl::onSceneWidgetStateChanged()
{
    editModeToggle->blockSignals(true);
    editModeToggle->setChecked(currentSceneWidget->isEditMode());
    editModeToggle->blockSignals(false);

    customModeButtonGroup.blockSignals(true);
    int customMode = currentSceneWidget->activeCustomMode();
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
        currentSceneWidget->viewpointOperationMode() != SceneWidget::ThirdPersonMode);
    firstPersonModeToggle->blockSignals(false);

    int polygonFlags = currentSceneWidget->visiblePolygonElements();
    
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
    highlightToggle->setChecked(currentSceneWidget->isHighlightingEnabled());
    highlightToggle->blockSignals(false);
    
    collisionLineToggle->blockSignals(true);
    collisionLineToggle->setChecked(currentSceneWidget->collisionLineVisibility());
    collisionLineToggle->blockSignals(false);
}


void SceneBar::Impl::onSceneRendererCamerasChanged()
{
    cameraCombo->blockSignals(true);
    
    vector<string> pathStrings;
    cameraCombo->clear();
    auto renderer = currentSceneWidget->renderer();
    const int n = renderer->numCameras();
    for(int i=0; i < n; ++i){
        renderer->getSimplifiedCameraPathStrings(i, pathStrings);
        string label;
        if(pathStrings.empty()){
            label = format("Camera {}", i);
        } else if(pathStrings.size() == 1){
            label = pathStrings.front();
        } else {
            label = pathStrings.back() + " - " + pathStrings.front();
        }
        cameraCombo->addItem(label.c_str());
    }
    cameraCombo->setCurrentIndex(renderer->currentCameraIndex());

    cameraCombo->blockSignals(false);
}


void SceneBar::Impl::onSceneRendererCurrentCameraChanged()
{
    cameraCombo->blockSignals(true);
    cameraCombo->setCurrentIndex(
        currentSceneWidget->renderer()->currentCameraIndex());
    cameraCombo->blockSignals(false);
}


void SceneBar::Impl::onCameraComboCurrentIndexChanged(int index)
{
    sceneViewConnections.block();
    currentSceneWidget->renderer()->setCurrentCamera(index);
    sceneViewConnections.unblock();
}


void SceneBar::Impl::onEditModeButtonToggled(bool on)
{
    sceneViewConnections.block();
    currentSceneWidget->setEditMode(on);
    sceneViewConnections.unblock();
}


void SceneBar::Impl::onFirstPersonModeButtonToggled(bool on)
{
    sceneViewConnections.block();
    currentSceneWidget->setViewpointOperationMode(
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
    currentSceneWidget->setVisiblePolygonElements(flags);
    
    sceneViewConnections.unblock();
}


void SceneBar::Impl::onHighlightingToggled(bool on)
{
    sceneViewConnections.block();
    currentSceneWidget->setHighlightingEnabled(on);
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
    auto renderer = currentSceneWidget->renderer();
    renderer->setProperty(SceneRenderer::PropertyKey("collisionDetectionModelVisibility"), mode);
    currentSceneWidget->renderScene();
}


void SceneBar::Impl::onCollisionLineButtonToggled(bool on)
{
    sceneViewConnections.block();
    currentSceneWidget->setCollisionLineVisibility(on);
    sceneViewConnections.unblock();
}


namespace {

class SceneCounter : public PolymorphicSceneNodeFunctionSet
{
public:
    int numVertices;
    int numNormals;
    int numTriangles;

    SceneCounter() {
        setFunction<SgGroup>(
            [&](SgGroup* group){
                for(auto child : *group){
                    dispatch(child);
                }
            });

        setFunction<SgShape>(
            [&](SgShape* shape){
                SgMesh* mesh = shape->mesh();
                if(mesh){
                    if(mesh->hasVertices()){
                        numVertices += mesh->vertices()->size();
                    }
                    if(mesh->hasNormals()){
                        numNormals += mesh->normals()->size();
                    }
                    numTriangles += mesh->numTriangles();
                }
            });

        setFunction<SgPointSet>(
            [&](SgPointSet* pointSet){
                if(pointSet->hasVertices()){
                    numVertices += pointSet->vertices()->size();
                }
            });

        updateDispatchTable();
    }
    
    void count(SgNode* node) {
        numVertices = 0;
        numNormals = 0;
        numTriangles = 0;
        dispatch(node);
    }
};

void putSceneStatistics()
{
    ostream& os = MessageView::instance()->cout();
    os << _("Scene statistics:") << endl;
    
    int numSceneItems = 0;
    int totalNumVertics = 0;
    int totalNumNormals = 0;
    int totalNumTriangles = 0;
    SceneCounter counter;

    for(auto& item : RootItem::instance()->selectedItems()){
        if(auto renderable = dynamic_cast<RenderableItem*>(item.get())){
            if(auto scene = renderable->getScene()){
                os << format(_(" Scene \"{}\":"), item->displayName()) << endl;
                counter.count(scene);
                os << format(_("  Vertices: {}\n"), counter.numVertices);
                os << format(_("  Normals: {}\n"), counter.numNormals);
                os << format(_("  Triangles: {}"), counter.numTriangles) << endl;
                totalNumVertics += counter.numVertices;
                totalNumNormals += counter.numNormals;
                totalNumTriangles += counter.numTriangles;
                ++numSceneItems;
            }
        }
    }

    if(!numSceneItems){
        os << _("No valid scene item is selected.") << endl;
    } else {
        os << format(_("The total number of {} scene items:\n"), numSceneItems);
        os << format(_(" Vertices: {}\n"), totalNumVertics);
        os << format(_(" Normals: {}\n"), totalNumNormals);
        os << format(_(" Triangles: {}"), totalNumTriangles) << endl;
    }
}

}
