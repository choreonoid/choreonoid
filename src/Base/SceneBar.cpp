/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneBar.h"
#include "SceneWidget.h"
#include <cnoid/ExtensionManager>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>
#include <cnoid/ComboBox>
#include <cnoid/ItemTreeView>
#include <cnoid/ConnectionSet>
#include <cnoid/SceneDrawables>
#include <cnoid/SceneProvider>
#include <cnoid/SceneRenderer>
#include <cnoid/AppConfig>
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
    ToolButton* wireframeToggle;
    ToolButton* visualModelToggle;
    ToolButton* modelTypeFlipButton;
    ToolButton* collisionModelToggle;
    ToolButton* collisionLineToggle;

    bool isCollisionVisualizationButtonSetVisible;

    MappingPtr config;

    SceneBarImpl(SceneBar* self);
    ~SceneBarImpl();
    void initialize();
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
    void onWireframeButtonToggled(bool on);
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
    impl = new SceneBarImpl(this);
    impl->initialize();
}


SceneBarImpl::SceneBarImpl(SceneBar* self)
    : self(self)
{

}


void SceneBarImpl::initialize()
{
    self->setVisibleByDefault(true);
    self->setEnabled(false);
    
    targetSceneWidget = 0;
    targetRenderer = 0;
    
    config = AppConfig::archive()->openMapping("SceneBar");
    
    editModeToggle = self->addToggleButton(
        QIcon(":/Base/icons/sceneedit.png"), _("Switch to the edit mode"));
    editModeToggle->sigToggled().connect(
        [&](bool on){ onEditModeButtonToggled(on); });

    firstPersonModeToggle = self->addToggleButton(
        QIcon(":/Base/icons/walkthrough.png"), _("First-person viewpoint control mode"));
    firstPersonModeToggle->sigToggled().connect(
        [&](bool on){ onFirstPersonModeButtonToggled(on); });

    cameraCombo = new ComboBox();
    cameraCombo->setToolTip(_("Select a camera"));
    cameraCombo->setMinimumContentsLength(6);
    cameraCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    cameraCombo->sigCurrentIndexChanged().connect(
        [&](int index){ onCameraComboCurrentIndexChanged(index); });
    self->addWidget(cameraCombo);

    self->addButton(QIcon(":/Base/icons/viewfitting.png"), _("Move the camera to look at the objects"))
        ->sigClicked().connect([&](){
                targetSceneWidget->viewAll();
                targetSceneWidget->setViewpointControlMode(SceneWidget::THIRD_PERSON_MODE);
            });

    wireframeToggle = self->addToggleButton(
        QIcon(":/Base/icons/wireframe.png"), _("Toggle the wireframe mode"));
    wireframeToggle->sigToggled().connect(
        [&](bool on){ onWireframeButtonToggled(on); });

    visualModelToggle = self->addToggleButton("V", _("Show visual models"));
    visualModelToggle->setChecked(true);
    visualModelToggle->sigToggled().connect(
        [&](bool){ updateCollisionModelVisibility(); });

    modelTypeFlipButton = self->addButton("F", _("Flip active model types"));
    modelTypeFlipButton->sigClicked().connect(
        [&](){ flipVisibleModels(); });

    collisionModelToggle = self->addToggleButton("C", _("Show the collision detection models"));
    collisionModelToggle->sigToggled().connect(
        [&](bool){ updateCollisionModelVisibility(); });
    
    collisionLineToggle = self->addToggleButton(
        QIcon(":/Base/icons/collisionlines.png"), _("Toggle the collision line visibility"));
    collisionLineToggle->sigToggled().connect(
        [&](bool on){ onCollisionLineButtonToggled(on); });

    isCollisionVisualizationButtonSetVisible = config->get("collisionButtonSet", false);
    self->setCollisionVisualizationButtonSetVisible(isCollisionVisualizationButtonSetVisible);
    
    self->addButton(QIcon(":/Base/icons/setup.png"), _("Show the config dialog"))
        ->sigClicked().connect([&](){ targetSceneWidget->showConfigDialog(); });

    SceneWidget::sigSceneWidgetCreated().connect(
        [&](SceneWidget* widget){ onSceneWidgetCreated(widget); });
}


SceneBar::~SceneBar()
{
    delete impl;
}


SceneBarImpl::~SceneBarImpl()
{
    config->write("collisionButtonSet", isCollisionVisualizationButtonSetVisible);
}


void SceneBarImpl::onSceneWidgetCreated(SceneWidget* sceneWidget)
{
    SceneWidgetInfo& info = sceneWidgetInfos[sceneWidget];

    info.connectionToSigFocusChanged =
        sceneWidget->sigWidgetFocusChanged().connect(
            [=](bool isFocused){ onSceneWidgetFocusChanged(sceneWidget, isFocused); });

    info.connectionToSigAboutToBeDestroyed =
        sceneWidget->sigAboutToBeDestroyed().connect(
            [=](){ onSceneWidgetAboutToBeDestroyed(sceneWidget); });

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
        targetRenderer = sceneWidget->renderer();

        onSceneWidgetStateChanged();

        sceneWidgetStateConnection =
            sceneWidget->sigStateChanged().connect(
                [&](){ onSceneWidgetStateChanged(); });

        onSceneRendererCamerasChanged();
        
        rendererStateConnections.add(
            targetRenderer->sigCamerasChanged().connect(
                [&](){ onSceneRendererCamerasChanged(); }));
        
        rendererStateConnections.add(
            targetRenderer->sigCurrentCameraChanged().connect(
                [&](){ onSceneRendererCurrentCameraChanged(); }));

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

    wireframeToggle->blockSignals(true);
    wireframeToggle->setChecked(targetSceneWidget->polygonMode() != SceneWidget::FILL_MODE);
    wireframeToggle->blockSignals(false);

    collisionLineToggle->blockSignals(true);
    collisionLineToggle->setChecked(targetSceneWidget->collisionLinesVisible());
    collisionLineToggle->blockSignals(false);
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


void SceneBarImpl::onWireframeButtonToggled(bool on)
{
    sceneWidgetStateConnection.block();
    targetSceneWidget->setPolygonMode(on ? SceneWidget::LINE_MODE : SceneWidget::FILL_MODE);
    sceneWidgetStateConnection.unblock();
}


bool SceneBar::isCollisionVisualizationButtonSetVisible() const
{
    return impl->isCollisionVisualizationButtonSetVisible;
}


void SceneBar::setCollisionVisualizationButtonSetVisible(bool on)
{
    impl->visualModelToggle->setVisible(on);
    impl->modelTypeFlipButton->setVisible(on);
    impl->collisionModelToggle->setVisible(on);
    impl->collisionLineToggle->setVisible(on);
    impl->isCollisionVisualizationButtonSetVisible = on;
}


void SceneBarImpl::flipVisibleModels()
{
    sceneWidgetStateConnection.block();
    visualModelToggle->toggle();
    collisionModelToggle->toggle();
    sceneWidgetStateConnection.unblock();
    updateCollisionModelVisibility();
}


void SceneBarImpl::updateCollisionModelVisibility()
{
    int mode = 0;
    if(visualModelToggle->isChecked()){
        mode = 1;
    }
    if(collisionModelToggle->isChecked()){
        mode += 2;
    }
    SceneRenderer* renderer = targetSceneWidget->renderer();
    renderer->setProperty(SceneRenderer::PropertyKey("collisionDetectionModelVisibility"), mode);
    renderer->sigRenderingRequest()();
}


void SceneBarImpl::onCollisionLineButtonToggled(bool on)
{
    sceneWidgetStateConnection.block();
    targetSceneWidget->setCollisionLinesVisible(on);
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
            label = format("Camera {}", i);
        } else if(pathStrings.size() == 1){
            label = pathStrings.front();
        } else {
            label = pathStrings.back() + " - " + pathStrings.front();
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

void SceneBar::sceneWidget(vector<SceneWidget*>& widget)
{
    for(auto it=impl->sceneWidgetInfos.begin(); it!=impl->sceneWidgetInfos.end(); it++){
        widget.push_back(it->first);
    }
}


namespace {

class SceneCounter : public PolymorphicFunctionSet<SgNode>
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

    ItemList<> selected = ItemTreeView::instance()->selectedItems();
    for(size_t i=0; i < selected.size(); ++i){
        Item* item = selected[i];
        SceneProvider* provider = dynamic_cast<SceneProvider*>(item);
        if(provider){
            SgNodePtr scene = provider->getScene();
            if(scene){
                os << format(_(" Scene \"{}\":"), item->name()) << endl;
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
