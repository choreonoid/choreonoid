/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidget.h"
#include "SceneBar.h"
#include "ToolBarArea.h"
#include "GL1SceneRenderer.h"
#include "GLSLSceneRenderer.h"
#include "SceneWidgetEditable.h"
#include "InteractiveCameraTransform.h"
#include "MainWindow.h"
#include "Buttons.h"
#include "ButtonGroup.h"
#include "CheckBox.h"
#include "ToolBar.h"
#include "Dialog.h"
#include "SpinBox.h"
#include "Separator.h"
#include "Archive.h"
#include "MessageView.h"
#include "MenuManager.h"
#include "Timer.h"
#include "LazyCaller.h"
#include "AppConfig.h"
#include <cnoid/Selection>
#include <cnoid/EigenArchive>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <cnoid/CoordinateAxesOverlay>
#include <cnoid/ConnectionSet>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QLabel>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <QColorDialog>
#include <QElapsedTimer>
#include <QMessageBox>
#include <QCoreApplication>
#include <QGridLayout>
#include <fmt/format.h>
#include <set>
#include <iostream>
#include "gettext.h"

#ifdef _WIN32
#undef near
#undef far
#endif

using namespace std;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;

const int NUM_SHADOWS = 2;

enum { FLOOR_GRID = 0, XZ_GRID = 1, YZ_GRID = 2 };

Signal<void()> sigVSyncModeChanged;

bool isLowMemoryConsumptionMode;
Signal<void(bool on)> sigLowMemoryConsumptionModeChanged;

class EditableExtractor : public PolymorphicFunctionSet<SgNode>
{
public:
    vector<SgNode*> editables;

    int numEditables() const { return editables.size(); }
    SgNode* editableNode(int index) { return editables[index]; }
    SceneWidgetEditable* editable(int index) { return dynamic_cast<SceneWidgetEditable*>(editables[index]); }

    EditableExtractor(){
        setFunction<SgGroup>(
            [&](SgNode* node){
                auto group = static_cast<SgGroup*>(node);
                if(dynamic_cast<SceneWidgetEditable*>(group)){
                    editables.push_back(group);
                }
                for(auto child : *group){
                    dispatch(child);
                }
            });
        updateDispatchTable();
    }

    void apply(SgNode* node) {
        editables.clear();
        dispatch(node);
    }
};

class ConfigDialog : public Dialog
{
public:
    SceneWidgetImpl* sceneWidgetImpl;
    QVBoxLayout* vbox;

    ScopedConnectionSet builtinCameraConnections;
    SpinBox fieldOfViewSpin;
    DoubleSpinBox zNearSpin;
    DoubleSpinBox zFarSpin;
    CheckBox restrictCameraRollCheck;
    ButtonGroup verticalAxisGroup;
    RadioButton verticalAxisZRadio;
    RadioButton verticalAxisYRadio;

    Selection lightingMode;
    ButtonGroup lightingModeGroup;
    RadioButton fullLightingRadio;
    RadioButton normalLightingRadio;
    RadioButton minLightingRadio;
    RadioButton solidColorLightingRadio;
    Selection cullingMode;
    ButtonGroup cullingModeGroup;
    RadioButton cullingRadios[GLSceneRenderer::N_CULLING_MODES];
    CheckBox smoothShadingCheck;
    CheckBox headLightCheck;
    DoubleSpinBox headLightIntensitySpin;
    CheckBox headLightFromBackCheck;
    CheckBox worldLightCheck;
    DoubleSpinBox worldLightIntensitySpin;
    DoubleSpinBox worldLightAmbientSpin;
    CheckBox additionalLightsCheck;
    CheckBox textureCheck;
    CheckBox fogCheck;
    struct Shadow {
        CheckBox check;
        QLabel lightLabel;
        SpinBox lightSpin;
    };
    Shadow shadows[NUM_SHADOWS];
    CheckBox shadowAntiAliasingCheck;
    CheckBox gridCheck[3];
    DoubleSpinBox gridSpanSpin[3];
    DoubleSpinBox gridIntervalSpin[3];
    PushButton backgroundColorButton;
    CheckBox coordinateAxesCheck;
    PushButton defaultColorButton;
    DoubleSpinBox pointSizeSpin;
    DoubleSpinBox lineWidthSpin;
    CheckBox pointRenderingModeCheck;
    Connection pointRenderingModeCheckConnection;
    CheckBox normalVisualizationCheck;
    DoubleSpinBox normalLengthSpin;
    CheckBox lightweightViewChangeCheck;
    //CheckBox fpsCheck;
    PushButton fpsTestButton;
    SpinBox fpsTestIterationSpin;
    CheckBox collisionVisualizationButtonsCheck;
    CheckBox upsideDownCheck;

    LazyCaller updateDefaultLightsLater;

    ConfigDialog(SceneWidgetImpl* impl);
    void showEvent(QShowEvent* event);
    void updateBuiltinCameraConfig();
    void storeState(Archive& archive);
    void restoreState(const Archive& archive);
};


class ImageWindow : public QWidget
{
    QLabel label;
    QPixmap pixmap;
public:
    ImageWindow(int width, int height)
        : QWidget(nullptr),
          pixmap(width, height)
    {
        setWindowTitle(_("OpengGL image buffer for picking"));
        auto box = new QHBoxLayout;
        box->setContentsMargins(0, 0, 0, 0);
        pixmap.setDevicePixelRatio(devicePixelRatio());
        label.setPixmap(pixmap);
        box->addWidget(&label);
        setLayout(box);
    }
    void setImage(const Image& image){
        if(image.numComponents() == 4){
            pixmap.convertFromImage(
                QImage(image.pixels(), image.width(), image.height(), QImage::Format_RGBA8888));
            pixmap.setDevicePixelRatio(devicePixelRatio());
            label.setPixmap(pixmap);
        }
    }
};
            
Signal<void(SceneWidget*)> sigSceneWidgetCreated;

}

namespace cnoid {

class SceneWidgetImpl : public QOpenGLWidget
{
    friend class SceneWidget;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneWidget* self;
    ostream& os;

    SceneWidgetRootPtr sceneRoot;
    SgGroupPtr systemGroup;
    SgGroup* scene;
    GLSceneRenderer* renderer;
    LazyCaller extractPreprocessedNodesLater;
    SgUpdate modified;
    SgUpdate added;
    SgUpdate removed;

    InteractiveCameraTransformPtr interactiveCameraTransform;
    InteractiveCameraTransformPtr builtinCameraTransform;
    SgPerspectiveCameraPtr builtinPersCamera;
    SgOrthographicCameraPtr builtinOrthoCamera;
    int numBuiltinCameras;
    bool isBuiltinCameraCurrent;
    bool isLightweightViewChangeEnabled;
    bool isCameraPositionInteractivelyChanged;
    Timer timerToRenderNormallyAfterInteractiveCameraPositionChange;

    SgDirectionalLightPtr worldLight;

    Signal<void()> sigStateChanged;
    LazyCaller emitSigStateChangedLater;

    bool needToUpdatedViewportInformation;
    bool isEditMode;

    Selection viewpointControlMode;
    bool isFirstPersonMode() const { return (viewpointControlMode.which() != SceneWidget::THIRD_PERSON_MODE); }
        
    enum DragMode { NO_DRAGGING, EDITING, VIEW_ROTATION, VIEW_TRANSLATION, VIEW_ZOOM } dragMode;

    bool isDraggingView() const {
        return (dragMode == VIEW_ROTATION ||
                dragMode == VIEW_TRANSLATION ||
                dragMode == VIEW_ZOOM);
    }

    set<SceneWidgetEditable*> editableEntities;

    typedef vector<SceneWidgetEditable*> EditablePath;
    EditablePath pointedEditablePath;
    SceneWidgetEditable* lastMouseMovedEditable;
    EditablePath focusedEditablePath;
    SceneWidgetEditable* focusedEditable;

    QCursor defaultCursor;
    QCursor editModeCursor;

    double orgMouseX;
    double orgMouseY;
    Vector3 orgPointedPos;
    Affine3 orgCameraPosition;
    double orgOrthoCameraHeight;
    Vector3 cameraViewChangeCenter;
        
    double dragAngleRatio;
    double viewTranslationRatioX;
    double viewTranslationRatioY;

    SceneWidgetEvent latestEvent;
    Vector3 lastClickedPoint;

    SceneWidgetEditable* eventFilter;
    ReferencedPtr eventFilterRef;

    Selection polygonMode;
    bool collisionLinesVisible;

    ref_ptr<CoordinateAxesOverlay> coordinateAxesOverlay;

    SgInvariantGroupPtr gridGroup;
    Vector4f gridColor[3];
    LazyCaller updateGridsLater;

    double fps;
    Timer fpsTimer;
    int fpsCounter;
    Timer fpsRenderingTimer;
    bool fpsRendered;
    bool isDoingFPSTest;
    bool isFPSTestCanceled;

    ConfigDialog* config;
    QLabel* indicatorLabel;

    MenuManager menuManager;
    Signal<void(const SceneWidgetEvent& event, MenuManager& menuManager)> sigContextMenuRequest;

    Signal<void(bool isFocused)> sigWidgetFocusChanged;
    Signal<void()> sigAboutToBeDestroyed;

    ImageWindow* pickingBufferImageWindow;

#ifdef ENABLE_SIMULATION_PROFILING
    int profiling_mode;
#endif

    static void onOpenGLVSyncToggled(bool on, bool doConfigOutput);
    static void onLowMemoryConsumptionModeChanged(bool on, bool doConfigOutput);

    SceneWidgetImpl(SceneWidget* self);
    ~SceneWidgetImpl();

    void onVSyncModeChanged();
    void onLowMemoryConsumptionModeChanged(bool on);

    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();

    void tryToResumeNormalRendering();

    void updateGrids();
    SgLineSet* createGrid(int index);

    void onSceneGraphUpdated(const SgUpdate& update);

    void showFPS(bool on);
    void onFPSTestButtonClicked();
    void doFPSTest();
    void onFPSUpdateRequest();
    void onFPSRenderingRequest();
    void renderFPS();

    void showBackgroundColorDialog();
    void showGridColorDialog(int index);
    void showDefaultColorDialog();

    void updateCurrentCamera();
    void setCurrentCameraPath(const std::vector<std::string>& simplifiedPathStrings);
    void onCamerasChanged();
    void onCurrentCameraChanged();

    void onTextureToggled(bool on);
    void onLineWidthChanged(double width);
    void onPointSizeChanged(double width);
    void setPolygonMode(int mode);
    void onPointRenderingModeToggled(bool on);
    void setCollisionLinesVisible(bool on);
    void onFieldOfViewChanged(double fov);
    void onClippingDepthChanged();
    void onSmoothShadingToggled(bool on);
    void updateDefaultLights();
    void onNormalVisualizationChanged();

    void resetCursor();
    void setEditMode(bool on);
    void toggleEditMode();
    void viewAll();

    void onEntityAdded(SgNode* node);
    void onEntityRemoved(SgNode* node);

    void showPickingBufferImageWindow();
    void onUpsideDownToggled(bool on);
        
    void updateLatestEvent(QKeyEvent* event);
    void updateLatestEvent(int x, int y, int modifiers);
    void updateLatestEvent(QMouseEvent* event);
    void updateLatestEventPath(bool forceFullPicking = false);
    void updateLastClickedPoint();
        
    SceneWidgetEditable* applyFunction(
        EditablePath& editablePath, std::function<bool(SceneWidgetEditable* editable)> function);
    bool setFocusToEditablePath(EditablePath& editablePath);
    bool setFocusToPointedEditablePath(SceneWidgetEditable* targetEditable);
    void clearFocusToEditables();

    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseDoubleClickEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    void updatePointerPosition();
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);
    virtual void focusInEvent(QFocusEvent* event);
    virtual void focusOutEvent(QFocusEvent* event);

    Affine3 getNormalizedCameraTransform(const Affine3& T);
    void startViewChange();
    void startViewRotation();
    void dragViewRotation();
    void startViewTranslation();
    void dragViewTranslation();
    void startViewZoom();
    void dragViewZoom();
    void zoomView(double ratio);
    void notifyCameraPositionInteractivelyChanged();

    void rotateBuiltinCameraView(double dPitch, double dYaw);
    void translateBuiltinCameraView(const Vector3& dp_local);

    void showViewModePopupMenu(const QPoint& globalPos);
    void showEditModePopupMenu(const QPoint& globalPos);

    void activateSystemNode(SgNode* node, bool on);

    bool saveImage(const std::string& filename);
    void setScreenSize(int width, int height);
    void updateIndicator(const std::string& text);
    bool storeState(Archive& archive);
    void writeCameraPath(Mapping& archive, const std::string& key, int cameraIndex);
    Mapping* storeCameraState(int cameraIndex, bool isInteractiveCamera, SgPosTransform* cameraTransform);
    bool restoreState(const Archive& archive);
    void restoreCameraStates(const Listing& cameraListing);
    int readCameraPath(const Mapping& archive, const char* key);
    void restoreCurrentCamera(const Mapping& cameraData);
};

}


void SceneWidget::initializeClass(ExtensionManager* ext)
{
    // OpenGL vsync setting
    Mapping* glConfig = AppConfig::archive()->openMapping("OpenGL");
    auto& mm = ext->menuManager();

    bool isVSyncEnabled = (glConfig->get("vsync", 0) > 0);
    auto vsyncItem = mm.setPath("/Options/OpenGL").addCheckItem(_("Vertical sync"));
    vsyncItem->setChecked(isVSyncEnabled);
    vsyncItem->sigToggled().connect([&](bool on){ SceneWidgetImpl::onOpenGLVSyncToggled(on, true); });
    SceneWidgetImpl::onOpenGLVSyncToggled(isVSyncEnabled, false);

    isLowMemoryConsumptionMode = glConfig->get("lowMemoryConsumption", false);
    auto memoryItem = mm.addCheckItem(_("Low GPU memory consumption mode"));
    memoryItem->setChecked(isLowMemoryConsumptionMode);
    memoryItem->sigToggled().connect([&](bool on){ SceneWidgetImpl::onLowMemoryConsumptionModeChanged(on, true); });
    SceneWidgetImpl::onLowMemoryConsumptionModeChanged(isVSyncEnabled, false);
}


void SceneWidgetImpl::onOpenGLVSyncToggled(bool on, bool doConfigOutput)
{
    auto format = QSurfaceFormat::defaultFormat();
    format.setSwapInterval(on ? 1 : 0);
    QSurfaceFormat::setDefaultFormat(format);
    sigVSyncModeChanged();

    if(doConfigOutput){
        Mapping* glConfig = AppConfig::archive()->openMapping("OpenGL");
        glConfig->write("vsync", (on ? 1 : 0));
    }
}


void SceneWidgetImpl::onLowMemoryConsumptionModeChanged(bool on, bool doConfigOutput)
{
    sigLowMemoryConsumptionModeChanged(on);

    if(doConfigOutput){
        Mapping* glConfig = AppConfig::archive()->openMapping("OpenGL");
        glConfig->write("lowMemoryConsumption", on);
    }
}


SceneWidgetRoot::SceneWidgetRoot(SceneWidget* sceneWidget)
    : sceneWidget_(sceneWidget)
{
    systemGroup = new SgGroup;
    systemGroup->setName("System");
    addChild(systemGroup);
}


static void extractPathsFromSceneWidgetRoot(SgNode* node, SgNodePath& reversedPath, vector<SgNodePath>& paths)
{
    reversedPath.push_back(node);
    if(!node->hasParents()){
        SceneWidgetRoot* sceneWidgetRoot = dynamic_cast<SceneWidgetRoot*>(node);
        if(sceneWidgetRoot){
            paths.push_back(reversedPath);
            std::reverse(paths.back().begin(), paths.back().end());
        }
    } else {
        SgObject::const_parentIter p;
        for(p = node->parentBegin(); p != node->parentEnd(); ++p){
            SgNode* node = dynamic_cast<SgNode*>(*p);
            if(node){
                extractPathsFromSceneWidgetRoot(node, reversedPath, paths);
            }
        }
    }
    reversedPath.pop_back();
}


void SceneWidget::forEachInstance(SgNode* node, std::function<void(SceneWidget* sceneWidget, const SgNodePath& path)> function)
{
    SgNodePath path;
    vector<SgNodePath> paths;
    extractPathsFromSceneWidgetRoot(node, path, paths);
    for(size_t i=0; i < paths.size(); ++i){
        const SgNodePath& path = paths[i];
        SceneWidgetRoot* root = static_cast<SceneWidgetRoot*>(path.front());
        function(root->sceneWidget(), path);
    }
}


SceneWidget::SceneWidget()
{
    impl = new SceneWidgetImpl(this);

    QVBoxLayout* vbox = new QVBoxLayout;
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->addWidget(impl);
    setLayout(vbox);

    ::sigSceneWidgetCreated(this);
}


SceneWidgetImpl::SceneWidgetImpl(SceneWidget* self)
    : QOpenGLWidget(self),
      self(self),
      os(MessageView::mainInstance()->cout()),
      sceneRoot(new SceneWidgetRoot(self)),
      systemGroup(sceneRoot->systemGroup),
      emitSigStateChangedLater(std::ref(sigStateChanged)),
      updateGridsLater([this](){ updateGrids(); })
{
    setFocusPolicy(Qt::WheelFocus);

    renderer = GLSceneRenderer::create(sceneRoot);
    if(auto glslRenderer = dynamic_cast<GLSLSceneRenderer*>(renderer)){
        glslRenderer->setLowMemoryConsumptionMode(isLowMemoryConsumptionMode);
    }
        
    renderer->setOutputStream(os);
    renderer->enableUnusedResourceCheck(true);
    renderer->sigRenderingRequest().connect([&](){ update(); });
    renderer->sigCamerasChanged().connect([&](){ onCamerasChanged(); });
    renderer->sigCurrentCameraChanged().connect([&](){ onCurrentCameraChanged(); });

    self->sigObjectNameChanged().connect([this](string name){ renderer->setName(name); });

    sceneRoot->sigUpdated().connect([this](const SgUpdate& update){ onSceneGraphUpdated(update); });

    scene = renderer->scene();

    extractPreprocessedNodesLater.setFunction([&](){ renderer->extractPreprocessedNodes(); });

    modified.setAction(SgUpdate::MODIFIED);
    added.setAction(SgUpdate::ADDED);
    removed.setAction(SgUpdate::REMOVED);

    for(auto& color : gridColor){
    	color << 0.9f, 0.9f, 0.9f, 1.0f;
    }

    setAutoFillBackground(false);
    setMouseTracking(true);

    needToUpdatedViewportInformation = true;
    isEditMode = false;
    viewpointControlMode.resize(2);
    viewpointControlMode.setSymbol(SceneWidget::THIRD_PERSON_MODE, "thirdPerson");
    viewpointControlMode.setSymbol(SceneWidget::FIRST_PERSON_MODE, "firstPerson");
    viewpointControlMode.select(SceneWidget::THIRD_PERSON_MODE);
    dragMode = NO_DRAGGING;
    defaultCursor = self->cursor();
    editModeCursor = QCursor(Qt::PointingHandCursor);

    lastMouseMovedEditable = 0;
    focusedEditable = 0;

    latestEvent.sceneWidget_ = self;
    lastClickedPoint.setZero();
    eventFilter = 0;
    
    indicatorLabel = new QLabel;
    indicatorLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    indicatorLabel->setAlignment(Qt::AlignLeft);
    QFont font = indicatorLabel->font();
    font.setFixedPitch(true);
    indicatorLabel->setFont(font);

    builtinCameraTransform = new InteractiveCameraTransform;
    builtinCameraTransform->setTransform(
        SgCamera::positionLookingAt(
            Vector3(4.0, 2.0, 1.5), Vector3(0.0, 0.0, 1.0), Vector3::UnitZ()));
    interactiveCameraTransform = builtinCameraTransform;

    builtinPersCamera = new SgPerspectiveCamera;
    builtinPersCamera->setName("Perspective");
    builtinPersCamera->setFieldOfView(radian(40.0));
    builtinCameraTransform->addChild(builtinPersCamera);

    builtinOrthoCamera = new SgOrthographicCamera;
    builtinOrthoCamera->setName("Orthographic");
    builtinOrthoCamera->setHeight(20.0f);
    builtinCameraTransform->addChild(builtinOrthoCamera);

    isBuiltinCameraCurrent = true;
    numBuiltinCameras = 2;
    systemGroup->addChild(builtinCameraTransform);
    isLightweightViewChangeEnabled = false;
    isCameraPositionInteractivelyChanged = false;
    timerToRenderNormallyAfterInteractiveCameraPositionChange.setSingleShot(true);
    timerToRenderNormallyAfterInteractiveCameraPositionChange.sigTimeout().connect(
        [&](){ tryToResumeNormalRendering(); });

    config = new ConfigDialog(this);
    config->updateBuiltinCameraConfig();

    worldLight = new SgDirectionalLight;
    worldLight->setName("WorldLight");
    worldLight->setDirection(Vector3(0.0, 0.0, -1.0));
    SgPosTransform* worldLightTransform = new SgPosTransform;
    worldLightTransform->setTranslation(Vector3(0.0, 0.0, 10.0));
    worldLightTransform->addChild(worldLight);
    systemGroup->addChild(worldLightTransform);
    renderer->setAsDefaultLight(worldLight);

    updateDefaultLights();

    polygonMode.resize(3);
    polygonMode.setSymbol(SceneWidget::FILL_MODE, "fill");
    polygonMode.setSymbol(SceneWidget::LINE_MODE, "line");
    polygonMode.setSymbol(SceneWidget::POINT_MODE, "point");
    polygonMode.select(SceneWidget::FILL_MODE);

    collisionLinesVisible = false;

    coordinateAxesOverlay = new CoordinateAxesOverlay;
    activateSystemNode(coordinateAxesOverlay, config->coordinateAxesCheck.isChecked());

    updateGrids();

    /*
    fpsTimer.sigTimeout().connect([&](){ onFPSUpdateRequest(); });
    fpsRenderingTimer.setSingleShot(true);
    fpsRenderingTimer.sigTimeout().connect([&](){ onFPSRenderingRequest(); });
    */

    isDoingFPSTest = false;

    sigVSyncModeChanged.connect([&](){ onVSyncModeChanged(); });
    sigLowMemoryConsumptionModeChanged.connect(
        [&](bool on){ onLowMemoryConsumptionModeChanged(on); });

    pickingBufferImageWindow = nullptr;

#ifdef ENABLE_SIMULATION_PROFILING
    profiling_mode = 1;
#endif
}


SceneWidget::~SceneWidget()
{
    sigAboutToBeDestroyed();
    delete impl;
}


SceneWidgetImpl::~SceneWidgetImpl()
{
    delete renderer;
    delete indicatorLabel;
    delete config;

    if(pickingBufferImageWindow){
        delete pickingBufferImageWindow;
    }
}


void SceneWidgetImpl::onVSyncModeChanged()
{
    /**
       We want to change the swap interval value when the vsync configuration is changed.
       However, resetting the format is not valied for the QOpenGLWidget instance that
       has been initialized.
       To change the swap interval, the QOpenGLWiget instance must probably be recreated.
    */
    //setFormat(QSurfaceFormat::defaultFormat());
}


void SceneWidgetImpl::onLowMemoryConsumptionModeChanged(bool on)
{
    if(auto glslRenderer = dynamic_cast<GLSLSceneRenderer*>(renderer)){
        glslRenderer->setLowMemoryConsumptionMode(on);
        update();
    }
}


SceneWidgetRoot* SceneWidget::sceneRoot()
{
    return impl->sceneRoot;
}


SgGroup* SceneWidget::scene()
{
    return impl->scene;
}


SceneRenderer* SceneWidget::renderer()
{
    return impl->renderer;
}


/**
   Draw the scene immediately.
   \note This function is only used to force rendering when testing rendering performance, etc.
*/
void SceneWidget::draw()
{
    impl->repaint();
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents|QEventLoop::ExcludeSocketNotifiers);
}


QWidget* SceneWidget::indicator()
{
    return impl->indicatorLabel;
}


void SceneWidgetImpl::initializeGL()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::initializeGL()" << endl;
    }

    if(!renderer->initializeGL()){
        os << "OpenGL initialization failed." << endl;
        // This view shoulbe be disabled when the glew initialization is failed.
    }
}


void SceneWidgetImpl::resizeGL(int width, int height)
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::resizeGL()" << endl;
    }
    needToUpdatedViewportInformation = true;
}


void SceneWidgetImpl::onSceneGraphUpdated(const SgUpdate& update)
{
    SgNode* node = dynamic_cast<SgNode*>(update.path().front());

    if(node && (update.action() & (SgUpdate::ADDED | SgUpdate::REMOVED))){
        EditableExtractor extractor;
        extractor.apply(node);
        const int numEditables = extractor.numEditables();

        if(update.action() & SgUpdate::ADDED){
            for(int i=0; i < numEditables; ++i){
                SceneWidgetEditable* editable = extractor.editable(i);
                editableEntities.insert(editable);
                editable->onSceneModeChanged(latestEvent);
            }
        } else if(update.action() & SgUpdate::REMOVED){
            for(int i=0; i < numEditables; ++i){
                SceneWidgetEditable* editable = extractor.editable(i);
                if(editable == lastMouseMovedEditable){
                    lastMouseMovedEditable->onPointerLeaveEvent(latestEvent);
                    lastMouseMovedEditable = 0;
                }
                bool isEntityFocused = false;
                for(size_t i=0; i < focusedEditablePath.size(); ++i){
                    if(editable == focusedEditablePath[i]){
                        isEntityFocused = true;
                    }
                }
                if(isEntityFocused){
                    clearFocusToEditables();
                }
                editableEntities.erase(editable);
            }
        }
    }
}


void SceneWidgetImpl::paintGL()
{
    if(TRACE_FUNCTIONS){
        static int counter = 0;
        os << "SceneWidgetImpl::paintGL() " << counter++ << endl;
    }

    if(needToUpdatedViewportInformation){
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        renderer->updateViewportInformation(viewport[0], viewport[1], viewport[2], viewport[3]);
        needToUpdatedViewportInformation = false;
    }

    bool isLightweightViewChangeActive = false;
    if(isLightweightViewChangeEnabled){
        isLightweightViewChangeActive = isCameraPositionInteractivelyChanged;
    }
    isCameraPositionInteractivelyChanged = false;
    
    renderer->setBoundingBoxRenderingForLightweightRenderingGroupEnabled(
        isLightweightViewChangeActive);

    if(isLightweightViewChangeActive){
        timerToRenderNormallyAfterInteractiveCameraPositionChange.start(
            isDraggingView() ? 0 : 400);
    } else if(timerToRenderNormallyAfterInteractiveCameraPositionChange.isActive()){
        timerToRenderNormallyAfterInteractiveCameraPositionChange.stop();
    }

    renderer->render();

    if(fpsTimer.isActive()){
        renderFPS();
    }

#ifdef ENABLE_SIMULATION_PROFILING
    renderer->setColor(Vector3f(1.0f, 1.0f, 1.0f));
    int n = self->profilingNames.size();
    if(self->profilingTimes.size() == n){
        QFont font("monospace");
        font.setStyleHint(QFont::Monospace);
        for(int i=0; i<n; i++){
            switch(profiling_mode){
            case 0:{
                double percentage;
                if(i!=n-1)
                    percentage = self->profilingTimes[i] / self->profilingTimes[n-1] * 100;
                else
                    percentage = self->profilingTimes[i] / self->worldTimeStep * 100;
                renderText(20, 20+20*i, QString::fromStdString(self->profilingNames[i]) + QString(": %1 %").arg(percentage,7,'f',1), font);
                }
                break;
            case 1:{
                renderText(20, 20+20*i, QString::fromStdString(self->profilingNames[i]) + QString(": %1 ns").arg(self->profilingTimes[i],9,'f',0), font);
                }
                break;
            case 2:{
                renderText(20, 20+20*i, QString::fromStdString(self->profilingNames[i]) + QString(": %1 micros").arg(self->profilingTimes[i]*1.0e-3,6,'f',0), font);
                }
                break;
            }
        }
    }
#endif

}


void SceneWidgetImpl::tryToResumeNormalRendering()
{
    if(isDraggingView()){
        timerToRenderNormallyAfterInteractiveCameraPositionChange.start(0);
    } else {
        update();
    }
}


void SceneWidgetImpl::renderFPS()
{
    /*
    renderer->setColor(Vector3f(1.0f, 1.0f, 1.0f));
    renderText(20, 20, QString("FPS: %1").arg(fps));
    */
    fpsRendered = true;
    ++fpsCounter;
}


void SceneWidgetImpl::onFPSUpdateRequest()
{
    double oldFps = fps;
    fps = fpsCounter / 0.5;
    fpsCounter = 0;
    fpsRendered = false;
    if(oldFps > 0.0 || fps > 0.0){
        fpsRenderingTimer.start(100);
    }
}
    

void SceneWidgetImpl::onFPSRenderingRequest()
{
    if(!fpsRendered){
        update();
        --fpsCounter;
    }
}


void SceneWidgetImpl::showFPS(bool on)
{
    if(on){
        fpsCounter = 0;
        fpsTimer.start(500);
    } else {
        fpsTimer.stop();
    }
    onFPSUpdateRequest();
}


void SceneWidgetImpl::onFPSTestButtonClicked()
{
    if(!isDoingFPSTest){
        auto& button = config->fpsTestButton;
        auto label = button.text();
        button.setText(_("Cancel"));
        doFPSTest();
        button.setText(label);
    } else {
        isFPSTestCanceled = true;
    }
}
        

void SceneWidgetImpl::doFPSTest()
{
    isDoingFPSTest = true;
    isFPSTestCanceled = false;
    
    const Vector3 p = lastClickedPoint;
    const Affine3 C = builtinCameraTransform->T();

    QElapsedTimer timer;
    timer.start();

    int numFrames = 0;
    const int n = config->fpsTestIterationSpin.value();
    for(int i=0; i < n; ++i){
        for(double theta=1.0; theta <= 360.0; theta += 1.0){
            double a = radian(theta);
            builtinCameraTransform->setTransform(
                Translation3(p) *
                AngleAxis(a, Vector3::UnitZ()) *
                Translation3(-p) *
                C);
            repaint();
            QCoreApplication::processEvents();
            ++numFrames;
            if(isFPSTestCanceled){
                break;
            }
        }
    }

    double time = timer.elapsed() / 1000.0;
    fps = numFrames / time;
    fpsCounter = 0;

    builtinCameraTransform->setTransform(C);
    update();

    QMessageBox::information(config, _("FPS Test Result"),
                             QString(_("FPS: %1 frames / %2 [s] = %3")).arg(numFrames).arg(time).arg(fps));

    isDoingFPSTest = false;
}


void SceneWidget::setCursor(const QCursor cursor)
{
    impl->setCursor(cursor);
}


void SceneWidgetImpl::resetCursor()
{
    setCursor(isEditMode ? editModeCursor : defaultCursor);
}


SignalProxy<void()> SceneWidget::sigStateChanged() const
{
    return impl->sigStateChanged;
}


void SceneWidget::setEditMode(bool on)
{
    impl->setEditMode(on);
}


bool SceneWidget::isEditMode() const
{
    return impl->isEditMode;
}


void SceneWidgetImpl::setEditMode(bool on)
{
    if(on != isEditMode){
        isEditMode = on;
        resetCursor();

        if(!isEditMode){
            for(size_t i=0; i < focusedEditablePath.size(); ++i){
                focusedEditablePath[i]->onFocusChanged(latestEvent, false);
            }
        }
        if(eventFilter){
            eventFilter->onSceneModeChanged(latestEvent);
        }
        set<SceneWidgetEditable*>::iterator p;
        for(p = editableEntities.begin(); p != editableEntities.end(); ++p){
            SceneWidgetEditable* editable = *p;
            editable->onSceneModeChanged(latestEvent);
        }
        if(isEditMode){
            for(size_t i=0; i < focusedEditablePath.size(); ++i){
                focusedEditablePath[i]->onFocusChanged(latestEvent, true);
            }
        }

        emitSigStateChangedLater();
    }
}


void SceneWidgetImpl::toggleEditMode()
{
    setEditMode(!isEditMode);
}


const SceneWidgetEvent& SceneWidget::latestEvent() const
{
    return impl->latestEvent;
}


Vector3 SceneWidget::lastClickedPoint() const
{
    return impl->lastClickedPoint;
}


void SceneWidget::setViewpointControlMode(ViewpointControlMode mode)
{
    impl->viewpointControlMode.select(mode);
    impl->emitSigStateChangedLater();
}


SceneWidget::ViewpointControlMode SceneWidget::viewpointControlMode() const
{
    return static_cast<SceneWidget::ViewpointControlMode>(impl->viewpointControlMode.which());
}


void SceneWidget::viewAll()
{
    impl->viewAll();
}


void SceneWidgetImpl::viewAll()
{
    if(!interactiveCameraTransform){
        return;
    }
    
    const BoundingBox& bbox = renderer->scene()->boundingBox();
    if(bbox.empty()){
        return;
    }
    const double radius = bbox.boundingSphereRadius();

    double left, right, bottom, top;
    renderer->getViewFrustum(builtinPersCamera, left, right, bottom, top);

    const double a = renderer->aspectRatio();
    double length = (a >= 1.0) ? (top - bottom) : (right - left);
    
    Affine3& T = interactiveCameraTransform->T();
    T.translation() +=
        (bbox.center() - T.translation())
        + T.rotation() * Vector3(0, 0, 2.0 * radius * builtinPersCamera->nearClipDistance() / length);


    if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(renderer->currentCamera())){
        if(a >= 1.0){
            ortho->setHeight(radius * 2.0);
        } else {
            ortho->setHeight(radius * 2.0 / a);
        }
        ortho->notifyUpdate(modified);
    }

    interactiveCameraTransform->notifyUpdate(modified);
}


void SceneWidgetImpl::showPickingBufferImageWindow()
{
    auto glslRenderer = dynamic_cast<GLSLSceneRenderer*>(renderer);
    if(glslRenderer){
        if(!pickingBufferImageWindow){
            auto vp = renderer->viewport();
            pickingBufferImageWindow = new ImageWindow(vp[2], vp[3]);
        }
        pickingBufferImageWindow->show();
    }
}


void SceneWidgetImpl::onUpsideDownToggled(bool on)
{
    renderer->setUpsideDown(on);
    update();
}


void SceneWidgetImpl::updateLatestEvent(QKeyEvent* event)
{
    latestEvent.modifiers_ = event->modifiers();
    latestEvent.key_ = event->key();
}


void SceneWidgetImpl::updateLatestEvent(int x, int y, int modifiers)
{
    latestEvent.x_ = x;
    latestEvent.y_ = height() - y - 1;
    latestEvent.modifiers_ = modifiers;
}


void SceneWidgetImpl::updateLatestEvent(QMouseEvent* event)
{
    updateLatestEvent(event->x(), event->y(), event->modifiers());
    latestEvent.button_ = event->button();
}


void SceneWidgetImpl::updateLatestEventPath(bool forceFullPicking)
{
    if(needToUpdatedViewportInformation ||
       (!forceFullPicking && isLightweightViewChangeEnabled)){
        return;
    }
    
    bool isPickingBufferVisualizationEnabled = pickingBufferImageWindow && pickingBufferImageWindow->isVisible();
    renderer->setPickingBufferImageOutputEnabled(isPickingBufferVisualizationEnabled);

    makeCurrent();

    const int r = devicePixelRatio();
    bool picked = renderer->pick(r * latestEvent.x(), r * latestEvent.y());

    if(isPickingBufferVisualizationEnabled){
        Image image;
        if(renderer->getPickingBufferImage(image)){
            pickingBufferImageWindow->setImage(image);
        }
    }

    doneCurrent();

    latestEvent.nodePath_.clear();
    pointedEditablePath.clear();

    if(picked){
        latestEvent.point_ = renderer->pickedPoint();
        latestEvent.nodePath_ = renderer->pickedNodePath();

        SgNodePath& path = latestEvent.nodePath_;
        for(size_t i=0; i < path.size(); ++i){
            SceneWidgetEditable* editable = dynamic_cast<SceneWidgetEditable*>(path[i]);
            if(editable){
                pointedEditablePath.push_back(editable);
            }
        }
    }
}


void SceneWidgetImpl::updateLastClickedPoint()
{
    const SgNodePath& path = latestEvent.nodePath();
    if(!path.empty()){
        if(!gridGroup || path.back() != gridGroup){
            lastClickedPoint = latestEvent.point();
        }
    }
}

    
/**
   \return The editable object with which the given function is actually applied (the function returns true.)
   If there are no functions which returns true, null object is returned.
*/
SceneWidgetEditable* SceneWidgetImpl::applyFunction
(EditablePath& editablePath, std::function<bool(SceneWidgetEditable* editable)> function)
{
    SceneWidgetEditable* targetEditable = 0;
    for(EditablePath::reverse_iterator p = editablePath.rbegin(); p != editablePath.rend(); ++p){
        SceneWidgetEditable* editable = *p;
        if(function(editable)){
            targetEditable = editable;
            break;
        }
    }
    return targetEditable;
}
    

bool SceneWidgetImpl::setFocusToEditablePath(EditablePath& editablePath)
{
    if(editablePath.empty()){
        return false;
    }
    
    int indexOfFirstEditableToChangeFocus = 0;
    for(size_t i=0; i < editablePath.size(); ++i){
        if(i < focusedEditablePath.size() && editablePath[i] == focusedEditablePath[i]){
            indexOfFirstEditableToChangeFocus = i + 1;
        } else {
            break;
        }
    }

    for(size_t i=indexOfFirstEditableToChangeFocus; i < focusedEditablePath.size(); ++i){
        focusedEditablePath[i]->onFocusChanged(latestEvent, false);
    }
    for(size_t i=indexOfFirstEditableToChangeFocus; i < editablePath.size(); ++i){
        editablePath[i]->onFocusChanged(latestEvent, true);
    }
    focusedEditablePath = editablePath;
    focusedEditable = editablePath.back();

    return true;
}


bool SceneWidgetImpl::setFocusToPointedEditablePath(SceneWidgetEditable* targetEditable)
{
    if(targetEditable){
        EditablePath path;
        for(size_t i=0; i < pointedEditablePath.size(); ++i){
            SceneWidgetEditable* editable = pointedEditablePath[i];
            path.push_back(editable);
            if(editable == targetEditable){
                return setFocusToEditablePath(path);
            }
        }
    }

    // No editable is pointed
    // The following command is disabled because keeping the focus seems better.
    // clearFocusToEditables();
    
    return false;
}


void SceneWidgetImpl::clearFocusToEditables()
{
    for(size_t i=0; i < focusedEditablePath.size(); ++i){
        focusedEditablePath[i]->onFocusChanged(latestEvent, false);
    }
    focusedEditablePath.clear();
    focusedEditable = 0;
}


bool SceneWidget::setSceneFocus(const SgNodePath& path)
{
    SceneWidgetImpl::EditablePath editablePath;
    for(size_t i=0; i < path.size(); ++i){
        SceneWidgetEditable* editable = dynamic_cast<SceneWidgetEditable*>(path[i]);
        if(editable){
            editablePath.push_back(editable);
        }
    }
    return impl->setFocusToEditablePath(editablePath);
}


void SceneWidgetImpl::keyPressEvent(QKeyEvent* event)
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::keyPressEvent()" << endl;
    }

    updateLatestEvent(event);
    updateLatestEventPath();

    bool handled = false;
    if(isEditMode){
        handled = applyFunction(
            focusedEditablePath,
            [&](SceneWidgetEditable* editable){ return editable->onKeyPressEvent(latestEvent); });
    }

    if(!handled){
        switch(event->key()){

        case Qt::Key_Escape:
            toggleEditMode();
            handled = true;
            break;
            
        case Qt::Key_Z:
            if(event->modifiers() & Qt::ControlModifier){
                if(event->modifiers() & Qt::ShiftModifier){
                    handled = applyFunction(
                        focusedEditablePath,
                        [&](SceneWidgetEditable* editable){ return editable->onRedoRequest(); });
                } else {
                    handled = applyFunction(
                        focusedEditablePath,
                        [&](SceneWidgetEditable* editable){ return editable->onUndoRequest(); });
                }
            }
            break;
            
        case Qt::Key_1:
            self->setViewpointControlMode(SceneWidget::FIRST_PERSON_MODE);
            handled = true;
            break;
            
        case Qt::Key_3:
            self->setViewpointControlMode(SceneWidget::THIRD_PERSON_MODE);
            handled = true;
            break;
            
        case Qt::Key_Space:
        {
            updateLastClickedPoint();
            latestEvent.button_ = Qt::MidButton;
            startViewChange();
            handled = true;
            break;
        }
#ifdef ENABLE_SIMULATION_PROFILING
        case Qt::Key_P:
            profiling_mode++;
            profiling_mode %= 3;
            break;
#endif
        default:
            break;
        }
    }

    if(!handled){
        event->setAccepted(false);
    }
}


void SceneWidgetImpl::keyReleaseEvent(QKeyEvent* event)
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::keyReleaseEvent()" << endl;
    }

    bool handled = false;
    
    updateLatestEvent(event);
    
    if(event->key() == Qt::Key_Space){
        dragMode = NO_DRAGGING;
        handled = true;

    } else if(dragMode == VIEW_ZOOM && (event->key() == Qt::Key_Control)){
        dragMode = NO_DRAGGING;
        handled = true;
        
    } else if(isEditMode){
        if(focusedEditable){
            handled = focusedEditable->onKeyReleaseEvent(latestEvent);
        }
    }

    if(!handled){
        event->setAccepted(false);
    }
}


void SceneWidgetImpl::mousePressEvent(QMouseEvent* event)
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::mousePressEvent()" << endl;
    }
    
    updateLatestEvent(event);
    updateLatestEventPath();
    updateLastClickedPoint();

    bool handled = false;
    bool forceViewMode = (event->modifiers() & Qt::AltModifier);

    if(isEditMode && !forceViewMode){
        if(event->button() == Qt::RightButton){
            showEditModePopupMenu(event->globalPos());
            handled = true;
        } else {
            if(eventFilter){
                handled = eventFilter->onButtonPressEvent(latestEvent);
            }
            if(!handled){
                handled = setFocusToPointedEditablePath(
                    applyFunction(
                        pointedEditablePath,
                        [&](SceneWidgetEditable* editable){ return editable->onButtonPressEvent(latestEvent); }));
            }
            if(handled){
                dragMode = EDITING;
            }
        }
    }

    if(!handled){
        if(!isEditMode && event->button() == Qt::RightButton){
            showViewModePopupMenu(event->globalPos());
            handled = true;
        }
    }

    if(!handled){
        startViewChange();
    }
}


void SceneWidgetImpl::mouseDoubleClickEvent(QMouseEvent* event)
{
    updateLatestEvent(event);
    updateLatestEventPath();
    
    bool handled = false;
    if(isEditMode){
        if(eventFilter){
            handled = eventFilter->onDoubleClickEvent(latestEvent);
        }
        if(!handled){
            handled = setFocusToPointedEditablePath(
                applyFunction(
                    pointedEditablePath,
                    [&](SceneWidgetEditable* editable){ return editable->onDoubleClickEvent(latestEvent); }));
        }
    }
    if(!handled){
        toggleEditMode();
    }
}


void SceneWidgetImpl::mouseReleaseEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    bool handled = false;
    if(isEditMode && dragMode == EDITING){
        if(eventFilter){
            handled = eventFilter->onButtonReleaseEvent(latestEvent);
        }
        if(!handled){
            if(focusedEditable){
                updateLatestEventPath();
                focusedEditable->onButtonReleaseEvent(latestEvent);
            }
        }
    }
    
    dragMode = NO_DRAGGING;
}


void SceneWidgetImpl::mouseMoveEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    bool handled = false;

    switch(dragMode){

    case EDITING:
        if(eventFilter){
            handled = eventFilter->onPointerMoveEvent(latestEvent);
        }
        if(!handled){
            if(focusedEditable){
                handled = focusedEditable->onPointerMoveEvent(latestEvent);
            }
        }
        break;

    case VIEW_ROTATION:
        dragViewRotation();
        break;
        
    case VIEW_TRANSLATION:
        dragViewTranslation();
        break;
        
    case VIEW_ZOOM:
        dragViewZoom();
        break;
        
    default:
        updatePointerPosition();
        break;
    }
}


static void findObjectNameFromChildren(SgObject* object, string& name)
{
    int n = object->numChildObjects();
    for(int i=0; i < n; ++i){
        SgObject* child = object->childObject(i);
        if(!child->name().empty()){
            name = child->name();
        } else {
            findObjectNameFromChildren(child, name);
        }
        if(!name.empty()){
            break;
        }
    }
}


static string findObjectNameFromNodePath(const SgNodePath& path)
{
    string name;

    if(!path.empty()){
        SgNode* node = path.back();
        if(node->name().empty()){
            findObjectNameFromChildren(node, name);
        }
        if(name.empty()){
            for(auto iter = path.rbegin(); iter != path.rend(); ++iter){
                SgNode* node = *iter;
                if(!node->name().empty()){
                    name = node->name();
                    break;
                }
            }
        }
    }

    return name;
}


void SceneWidgetImpl::updatePointerPosition()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::updatePointerPosition()" << endl;
    }

    updateLatestEventPath(true);
    
    if(!isEditMode){
        static string f1(_("Global Position: ({0:.3f} {1:.3f} {2:.3f})"));
        static string f2(_("Object: {0}, Global Position: ({1:.3f} {2:.3f} {3:.3f})"));
        const Vector3& p = latestEvent.point();
        string name = findObjectNameFromNodePath(latestEvent.nodePath());
        if(name.empty()){
            updateIndicator(fmt::format(f1, p.x(), p.y(), p.z()));
        } else {
            updateIndicator(fmt::format(f2, name, p.x(), p.y(), p.z()));
        }
        
    } else {
        SceneWidgetEditable* mouseMovedEditable = applyFunction(
            pointedEditablePath,
            [&](SceneWidgetEditable* editable){ return editable->onPointerMoveEvent(latestEvent); });

        if(mouseMovedEditable){
            if(!QWidget::hasFocus()){
                QWidget::setFocus(Qt::MouseFocusReason);
            }
        }
        if(lastMouseMovedEditable != mouseMovedEditable){
            if(!mouseMovedEditable){
                resetCursor();
            }
            if(lastMouseMovedEditable){
                lastMouseMovedEditable->onPointerLeaveEvent(latestEvent);
            }
            lastMouseMovedEditable = mouseMovedEditable;
        }
    }
}


void SceneWidgetImpl::wheelEvent(QWheelEvent* event)
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::wheelEvent()" << endl;
        os << "delta: " << event->delta() << endl;
    }

    updateLatestEvent(event->x(), event->y(), event->modifiers());
    updateLatestEventPath();
    updateLastClickedPoint();

    const double s = event->delta() / 8.0 / 15.0;
    latestEvent.wheelSteps_ = s;

    bool handled = false;
    if(isEditMode){
        if(eventFilter){
            handled = eventFilter->onScrollEvent(latestEvent);
        }
        if(!handled){
            handled = setFocusToPointedEditablePath(
                applyFunction(
                    pointedEditablePath,
                    [&](SceneWidgetEditable* editable){ return editable->onScrollEvent(latestEvent); }));
        }
    }    

    if(interactiveCameraTransform){
        if(!handled || !isEditMode){
            if(event->orientation() == Qt::Vertical){
                zoomView(0.25 * s);
            }
        }
    }
}


bool SceneWidget::unproject(double x, double y, double z, Vector3& out_projected) const
{
    const int r = devicePixelRatio();
    return impl->renderer->unproject(r * x, r * y, z, out_projected);
}


SignalProxy<void(SceneWidget*)> SceneWidget::sigSceneWidgetCreated()
{
    return ::sigSceneWidgetCreated;
}


SignalProxy<void(bool isFocused)> SceneWidget::sigWidgetFocusChanged()
{
    return impl->sigWidgetFocusChanged;
}


void SceneWidgetImpl::focusInEvent(QFocusEvent*)
{
    sigWidgetFocusChanged(true);
}


void SceneWidgetImpl::focusOutEvent(QFocusEvent*)
{
    sigWidgetFocusChanged(false);
}


SignalProxy<void()> SceneWidget::sigAboutToBeDestroyed()
{
    return impl->sigAboutToBeDestroyed;
}


/**
   \note Z axis should always be the upper vertical direciton.
*/
Affine3 SceneWidgetImpl::getNormalizedCameraTransform(const Affine3& T)
{
    if(!config->restrictCameraRollCheck.isChecked()){
        return T;
    }
    
    Vector3 verticalAxis;
    if(config->verticalAxisZRadio.isChecked()){
        verticalAxis = Vector3::UnitZ();
    } else {
        verticalAxis = Vector3::UnitY();
    }
    
    Vector3 x, y;
    Vector3 z = T.linear().col(2).normalized();
    if(fabs(z.dot(verticalAxis) > 0.9)){
        x = T.linear().col(0).normalized();
    } else {
        y = T.linear().col(1);
        if(y.dot(verticalAxis) >= 0.0){
            x = verticalAxis.cross(z).normalized();
        } else {
            x = z.cross(verticalAxis).normalized();
        }
    }
    y = z.cross(x);
        
    Affine3 N;
    N.linear() << x, y, z;
    N.translation() = T.translation();
    return N;
}


void SceneWidgetImpl::startViewChange()
{
    if(interactiveCameraTransform){

        switch(latestEvent.button()){
                
        case Qt::LeftButton:
            startViewRotation();
            break;
            
        case Qt::MidButton:
            if(latestEvent.modifiers() & Qt::ControlModifier){
                startViewZoom();
            } else {
                startViewTranslation();
            }
            break;
            
        default:
            break;
        }
    }
}
       

void SceneWidgetImpl::startViewRotation()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::startViewRotation()" << endl;
    }

    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }
    
    orgMouseX = latestEvent.x();
    orgMouseY = latestEvent.y();
    orgCameraPosition = interactiveCameraTransform->T();

    if(isFirstPersonMode()){
        orgPointedPos = orgCameraPosition.translation();
        dragAngleRatio = 0.01f;
    } else {
        orgPointedPos = lastClickedPoint;
        dragAngleRatio = 0.01f;
    }
    
    dragMode = VIEW_ROTATION;
}


void SceneWidgetImpl::dragViewRotation()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::dragViewRotation()" << endl;
    }

    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }
    
    const double dx = latestEvent.x() - orgMouseX;
    const double dy = latestEvent.y() - orgMouseY;

    Affine3 R;
    if(config->restrictCameraRollCheck.isChecked()){
        Vector3 up;
        if(config->verticalAxisZRadio.isChecked()){
            up = Vector3::UnitZ();
        } else {
            up = Vector3::UnitY();
        }
        R = AngleAxis(-dx * dragAngleRatio, up) *
            AngleAxis(dy * dragAngleRatio, SgCamera::right(orgCameraPosition));
    } else {
        if(latestEvent.modifiers() & Qt::ControlModifier){
            R = AngleAxis(-dx * dragAngleRatio, SgCamera::direction(orgCameraPosition));
        } else {
            R = AngleAxis(-dx * dragAngleRatio, SgCamera::up(orgCameraPosition)) *
                AngleAxis(dy * dragAngleRatio, SgCamera::right(orgCameraPosition));
        }
    }
        
    interactiveCameraTransform->setTransform(
        getNormalizedCameraTransform(
            Translation3(orgPointedPos) *
            R *
            Translation3(-orgPointedPos) *
            orgCameraPosition));

    if(latestEvent.modifiers() & Qt::ShiftModifier){
        Affine3& T = interactiveCameraTransform->T();
        Vector3 rpy = rpyFromRot(T.linear());
        for(int i=0; i < 3; ++i){
            double& a = rpy[i];
            for(int j=0; j < 5; ++j){
                double b = j * (PI / 2.0) - PI;
                if(fabs(b - a) < (PI / 8)){
                    a = b;
                    break;
                }
            }
        }
        Matrix3 S = rotFromRpy(rpy);
        T.translation() = orgPointedPos + S * T.linear().transpose() * (T.translation() - orgPointedPos);
        T.linear() = S;
    }
    
    notifyCameraPositionInteractivelyChanged();
}


void SceneWidgetImpl::startViewTranslation()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::startViewTranslation()" << endl;
    }

    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }

    const Affine3& C = interactiveCameraTransform->T();

    if(isFirstPersonMode()){
        viewTranslationRatioX = -0.005;
        viewTranslationRatioY = -0.005;

    } else {
        int x, y, width, height;
        renderer->getViewport(x, y, width, height);
        const double aspect = (double)width / height;
        double r{}, cw{}, ch{};
        SgCamera* camera = renderer->currentCamera();
        if(SgPerspectiveCamera* pers = dynamic_cast<SgPerspectiveCamera*>(camera)){
            const double fovy = pers->fovy(aspect);
            r = (lastClickedPoint - C.translation()).dot(SgCamera::direction(C));
            ch = tanf(fovy / 2.0) * 2.0;
            cw = aspect * ch;
        } else if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
            r = 1.0;
            ch = ortho->height();
            cw = aspect * ch;
        }
        viewTranslationRatioX = r * cw / width;
        viewTranslationRatioY = r * ch / height;
    }
    
    orgMouseX = latestEvent.x();
    orgMouseY = latestEvent.y();
    orgCameraPosition = C;
    dragMode = VIEW_TRANSLATION;
}


void SceneWidgetImpl::dragViewTranslation()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::dragViewTranslation()" << endl;
    }
    
    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }

    const double dx = viewTranslationRatioX * (latestEvent.x() - orgMouseX);
    const double dy = viewTranslationRatioY * (latestEvent.y() - orgMouseY);

    interactiveCameraTransform->setTransform(
        getNormalizedCameraTransform(
            Translation3(-dy * SgCamera::up(orgCameraPosition)) *
            Translation3(-dx * SgCamera::right(orgCameraPosition)) *
            orgCameraPosition));
    
    notifyCameraPositionInteractivelyChanged();
}


void SceneWidgetImpl::startViewZoom()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::startViewZoom()" << endl;
    }

    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }

    orgMouseY = latestEvent.y();
    orgCameraPosition = interactiveCameraTransform->T();

    if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(renderer->currentCamera())){
        orgOrthoCameraHeight = ortho->height();
    }
    
    dragMode = VIEW_ZOOM;
}


void SceneWidgetImpl::dragViewZoom()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::dragViewZoom()" << endl;
    }

    SgCamera* camera = renderer->currentCamera();
    
    const double dy = latestEvent.y() - orgMouseY;
    const double ratio = expf(dy * 0.01);

    if(dynamic_cast<SgPerspectiveCamera*>(camera)){
        const Affine3& C = orgCameraPosition;
        const Vector3 v = SgCamera::direction(C);

        if(isFirstPersonMode()){
            double speed = 0.02;
            interactiveCameraTransform->setTranslation(C.translation() + speed * dy * v);
            
        } else {
            const double l0 = (lastClickedPoint - C.translation()).dot(v);
            interactiveCameraTransform->setTranslation(C.translation() + v * (l0 * (-ratio + 1.0)));
        }

    } else if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
        ortho->setHeight(orgOrthoCameraHeight * ratio);
        ortho->notifyUpdate(modified);
    }

    notifyCameraPositionInteractivelyChanged();
}


void SceneWidgetImpl::zoomView(double ratio)
{
    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }

    SgCamera* camera = renderer->currentCamera();
    if(dynamic_cast<SgPerspectiveCamera*>(camera)){
        const Affine3& C = interactiveCameraTransform->T();
        const Vector3 v = SgCamera::direction(C);
        
        if(isFirstPersonMode()){
            if(latestEvent.modifiers() & Qt::ShiftModifier){
                ratio *= 5.0;
            }
            interactiveCameraTransform->translation() += ratio * v;

        } else {
            if(latestEvent.modifiers() & Qt::ShiftModifier){
                ratio *= 0.2;
            }
            const double dz = ratio * (lastClickedPoint - C.translation()).dot(v);
            interactiveCameraTransform->translation() -= dz * v;
        }

    } else if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
        ortho->setHeight(ortho->height() * expf(ratio));
        ortho->notifyUpdate(modified);
    }

    notifyCameraPositionInteractivelyChanged();
}


void SceneWidgetImpl::notifyCameraPositionInteractivelyChanged()
{
    isCameraPositionInteractivelyChanged = true;
    interactiveCameraTransform->notifyUpdate(modified);
}
    

void SceneWidget::startBuiltinCameraViewChange(const Vector3& center)
{
    impl->orgCameraPosition = impl->builtinCameraTransform->T();
    impl->cameraViewChangeCenter = center;
}


void SceneWidget::rotateBuiltinCameraView(double dPitch, double dYaw)
{
    impl->rotateBuiltinCameraView(dPitch, dYaw);
}


void SceneWidgetImpl::rotateBuiltinCameraView(double dPitch, double dYaw)
{
    const Affine3 T = builtinCameraTransform->T();
    builtinCameraTransform->setTransform(
        getNormalizedCameraTransform(
            Translation3(cameraViewChangeCenter) *
            AngleAxis(dYaw, Vector3::UnitZ()) *
            AngleAxis(dPitch, SgCamera::right(T)) *
            Translation3(-cameraViewChangeCenter) *
            T));

    builtinCameraTransform->notifyUpdate(modified);
}


void SceneWidget::translateBuiltinCameraView(const Vector3& dp_local)
{
    impl->translateBuiltinCameraView(dp_local);
}


void SceneWidgetImpl::translateBuiltinCameraView(const Vector3& dp_local)
{
    const Affine3 T = builtinCameraTransform->T();
    builtinCameraTransform->setTransform(
        getNormalizedCameraTransform(
            Translation3(T.linear() * dp_local) * T));
    builtinCameraTransform->notifyUpdate(modified);
}


void SceneWidgetImpl::showViewModePopupMenu(const QPoint& globalPos)
{
    menuManager.setNewPopupMenu(this);
    
    sigContextMenuRequest(latestEvent, menuManager);

    menuManager.setPath("/");
    menuManager.addItem(_("Edit Mode")) ->sigTriggered().connect([&](){ toggleEditMode(); });

    menuManager.popupMenu()->popup(globalPos);
}


void SceneWidgetImpl::showEditModePopupMenu(const QPoint& globalPos)
{
    menuManager.setNewPopupMenu(this);

    int prevNumItems = 0;

    if(!pointedEditablePath.empty()){
        SceneWidgetEditable* editableToFocus = pointedEditablePath.front();
        for(EditablePath::reverse_iterator p = pointedEditablePath.rbegin(); p != pointedEditablePath.rend(); ++p){
            SceneWidgetEditable* editable = *p;
            editable->onContextMenuRequest(latestEvent, menuManager);
            int numItems = menuManager.numItems();
            if(numItems > prevNumItems){
                menuManager.addSeparator();
                prevNumItems = numItems;
                editableToFocus = editable;
            }
        }
        setFocusToPointedEditablePath(editableToFocus);
    }

    if(eventFilter){
        eventFilter->onContextMenuRequest(latestEvent, menuManager);
        if(menuManager.numItems() > prevNumItems){
            menuManager.addSeparator();
            prevNumItems = menuManager.numItems();
        }
    }

    sigContextMenuRequest(latestEvent, menuManager);
    if(menuManager.numItems() > prevNumItems){
        menuManager.addSeparator();
    }

    menuManager.setPath("/");
    menuManager.addItem(_("View Mode"))->sigTriggered().connect([&](){ toggleEditMode(); });
    
    menuManager.popupMenu()->popup(globalPos);
}


Menu* SceneWidget::contextMenu()
{
    return impl->menuManager.popupMenu();
}


void SceneWidget::showContextMenu()
{
    QPoint pos = QWidget::mapToGlobal(QPoint(0, 0));
    if(impl->isEditMode){
        impl->showEditModePopupMenu(pos);
    } else {
        impl->showViewModePopupMenu(pos);
    }
}


SignalProxy<void(const SceneWidgetEvent& event, MenuManager& menuManager)>
SceneWidget::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}


void SceneWidget::installEventFilter(SceneWidgetEditable* filter)
{
    impl->eventFilter = filter;
    impl->eventFilterRef = dynamic_cast<Referenced*>(filter);
}


SceneWidgetEditable* SceneWidget::activeEventFilter()
{
    return impl->eventFilter;
}


void SceneWidget::removeEventFilter(SceneWidgetEditable* filter)
{
    if(impl->eventFilter == filter){
        impl->eventFilter = 0;
        impl->eventFilterRef.reset();
        impl->resetCursor();
    }
}


void SceneWidgetImpl::showBackgroundColorDialog()
{
    const Vector3f& c = renderer->backgroundColor();
    QColor newColor =
        QColorDialog::getColor(
            QColor::fromRgbF(c[0], c[1], c[2], 1.0f),
            MainWindow::instance(), _("Background Color"));
    
    if(newColor.isValid()){
        renderer->setBackgroundColor(Vector3f(newColor.redF(), newColor.greenF(), newColor.blueF()));
        update();
    }
}


void SceneWidgetImpl::showGridColorDialog(int index)
{
    const Vector4f& c = gridColor[index];
    QColor newColor = QColorDialog::getColor(
        QColor::fromRgbF(c[0], c[1], c[2], c[3]),
        MainWindow::instance(), _("Floor Grid Color"));
    
    if(newColor.isValid()){
        gridColor[index] << newColor.redF(), newColor.greenF(), newColor.blueF(), newColor.alphaF();
        updateGrids();
    }
}


void SceneWidgetImpl::showDefaultColorDialog()
{
    const Vector3f& dc = renderer->defaultColor();
    QColor c = QColorDialog::getColor(
        QColor::fromRgbF(dc[0], dc[1], dc[2]),
        MainWindow::instance(), _("Default Color"));
    
    if(c.isValid()){
        Vector3f color(c.redF(), c.greenF(), c.blueF());
        renderer->setDefaultColor(color);
        renderer->defaultMaterial()->setDiffuseColor(color);
        renderer->requestToClearResources();
        update();
    }
}


void SceneWidgetImpl::updateCurrentCamera()
{
    const int index = renderer->currentCameraIndex();
    if(index >= 0){
        latestEvent.cameraPath_ = renderer->cameraPath(index);
    }
}
    
        
SgPosTransform* SceneWidget::builtinCameraTransform()
{
    return impl->builtinCameraTransform;
}


SgPerspectiveCamera* SceneWidget::builtinPerspectiveCamera() const
{
    return impl->builtinPersCamera;
}


SgOrthographicCamera* SceneWidget::builtinOrthographicCamera() const
{
    return impl->builtinOrthoCamera;
}


bool SceneWidget::isBuiltinCameraCurrent() const
{
    return impl->isBuiltinCameraCurrent;
}


bool SceneWidget::isBuiltinCamera(SgCamera* camera) const
{
    return (camera == impl->builtinPersCamera || camera == impl->builtinOrthoCamera);
}


void SceneWidgetImpl::setCurrentCameraPath(const std::vector<std::string>& simplifiedPathStrings)
{
    renderer->setCurrentCameraPath(simplifiedPathStrings);
    updateCurrentCamera();
}


InteractiveCameraTransform* SceneWidget::findOwnerInteractiveCameraTransform(int cameraIndex)
{
    const SgNodePath& path = impl->renderer->cameraPath(cameraIndex);
    for(size_t i=0; i < path.size() - 1; ++i){
        if(InteractiveCameraTransform* transform = dynamic_cast<InteractiveCameraTransform*>(path[i])){
            return transform;
        }
    }
    return 0;
}


void SceneWidgetImpl::onCamerasChanged()
{
    updateCurrentCamera();
}


void SceneWidgetImpl::onCurrentCameraChanged()
{
    interactiveCameraTransform.reset();
    isBuiltinCameraCurrent = false;
    
    SgCamera* current = renderer->currentCamera();
    if(current){
        int index = renderer->currentCameraIndex();
        const SgNodePath& path = renderer->cameraPath(index);
        for(int i = path.size() - 2; i >= 0; --i){
            interactiveCameraTransform = dynamic_cast<InteractiveCameraTransform*>(path[i]);
            if(interactiveCameraTransform){
                isBuiltinCameraCurrent = (current == builtinPersCamera || current == builtinOrthoCamera);
                break;
            }
        }
    }
}


void SceneWidgetImpl::onTextureToggled(bool on)
{
    renderer->enableTexture(on);
    update();
}


void SceneWidgetImpl::onLineWidthChanged(double width)
{
    renderer->setDefaultLineWidth(width);
    update();
}


void SceneWidgetImpl::onPointSizeChanged(double size)
{
    renderer->setDefaultPointSize(size);
    update();
}


void SceneWidget::setPolygonMode(PolygonMode mode)
{
    impl->setPolygonMode(mode);
}


void SceneWidgetImpl::setPolygonMode(int mode)
{
    if(mode < 0 && mode > 2){
        return;
    }
    
    int oldMode = polygonMode.which();

    if(mode == SceneWidget::POINT_MODE){
        config->pointRenderingModeCheckConnection.block();
        config->pointRenderingModeCheck.setChecked(true);
        config->pointRenderingModeCheckConnection.unblock();
    }
    if(mode == SceneWidget::LINE_MODE && config->pointRenderingModeCheck.isChecked()){
        polygonMode.select(SceneWidget::POINT_MODE);
    } else {
        polygonMode.select(mode);
    }

    if(polygonMode.which() != oldMode){
        switch(polygonMode.which()){
        case SceneWidget::FILL_MODE:
            renderer->setPolygonMode(GL1SceneRenderer::FILL_MODE);
            break;
        case SceneWidget::LINE_MODE:
            renderer->setPolygonMode(GL1SceneRenderer::LINE_MODE);
            break;
        case SceneWidget::POINT_MODE:
            renderer->setPolygonMode(GL1SceneRenderer::POINT_MODE);
            break;
        default:
            break;
        }
        update();
    }
}
    
    
SceneWidget::PolygonMode SceneWidget::polygonMode() const
{
    return static_cast<SceneWidget::PolygonMode>(impl->polygonMode.which());
}


void SceneWidgetImpl::onPointRenderingModeToggled(bool on)
{
    if(on){
        setPolygonMode(polygonMode.which());
    } else {
        if(polygonMode.which() == SceneWidget::POINT_MODE){
            setPolygonMode(SceneWidget::LINE_MODE);
        }
    }
}


void SceneWidget::setCollisionLinesVisible(bool on)
{
    impl->setCollisionLinesVisible(on);
}


void SceneWidgetImpl::setCollisionLinesVisible(bool on)
{
    if(on != collisionLinesVisible){
        collisionLinesVisible = on;
        renderer->setProperty(SceneRenderer::PropertyKey("collisionLineRatio"), on ? 50.0 : 0.0);
        update();
        emitSigStateChangedLater();
    }
}


bool SceneWidget::collisionLinesVisible() const
{
    return impl->collisionLinesVisible;
}


void SceneWidgetImpl::onFieldOfViewChanged(double fov)
{
    config->builtinCameraConnections.block();
    builtinPersCamera->setFieldOfView(fov);
    builtinPersCamera->notifyUpdate(modified);
    config->builtinCameraConnections.unblock();
}


void SceneWidgetImpl::onClippingDepthChanged()
{
    config->builtinCameraConnections.block();
    double zNear = config->zNearSpin.value();
    double zFar = config->zFarSpin.value();
    builtinPersCamera->setNearClipDistance(zNear);
    builtinPersCamera->setFarClipDistance(zFar);
    builtinOrthoCamera->setNearClipDistance(zNear);
    builtinOrthoCamera->setFarClipDistance(zFar);
    builtinPersCamera->notifyUpdate(modified);
    builtinOrthoCamera->notifyUpdate(modified);
    config->builtinCameraConnections.unblock();
}


void SceneWidgetImpl::onSmoothShadingToggled(bool on)
{
    renderer->setDefaultSmoothShading(on);
    update();
}


void SceneWidgetImpl::updateDefaultLights()
{
    SgLight* headLight = renderer->headLight();
    headLight->on(config->headLightCheck.isChecked());
    headLight->setIntensity(config->headLightIntensitySpin.value());

    GL1SceneRenderer* gl1Renderer = dynamic_cast<GL1SceneRenderer*>(renderer);
    if(gl1Renderer){
        gl1Renderer->setHeadLightLightingFromBackEnabled(config->headLightFromBackCheck.isChecked());
    }

    worldLight->on(config->worldLightCheck.isChecked());
    worldLight->setIntensity(config->worldLightIntensitySpin.value());
    worldLight->setAmbientIntensity(config->worldLightAmbientSpin.value());

    renderer->enableAdditionalLights(config->additionalLightsCheck.isChecked());

    renderer->clearShadows();
    for(int i=0; i < NUM_SHADOWS; ++i){
        ConfigDialog::Shadow& s = config->shadows[i];
        if(s.check.isChecked()){
            renderer->enableShadowOfLight(s.lightSpin.value(), true);
        }
    }
    renderer->enableShadowAntiAliasing(config->shadowAntiAliasingCheck.isChecked());

    renderer->enableFog(config->fogCheck.isChecked());

    worldLight->notifyUpdate(modified);
}


void SceneWidgetImpl::onNormalVisualizationChanged()
{
    if(config->normalVisualizationCheck.isChecked()){
        renderer->showNormalVectors(config->normalLengthSpin.value());
    } else {
        renderer->showNormalVectors(0.0);
    }
    update();
}


void SceneWidget::setHeadLightIntensity(double value)
{
    impl->config->headLightIntensitySpin.setValue(value);
}


void SceneWidget::setWorldLightIntensity(double value)
{
    impl->config->worldLightIntensitySpin.setValue(value);
}


void SceneWidget::setWorldLightAmbient(double value)
{
    impl->config->worldLightAmbientSpin.setValue(value);
}


void SceneWidget::setFloorGridSpan(double value)
{
    impl->config->gridSpanSpin[FLOOR_GRID].setValue(value);
}


void SceneWidget::setFloorGridInterval(double value)
{
    impl->config->gridIntervalSpin[FLOOR_GRID].setValue(value);
}


void SceneWidget::setLineWidth(double value)
{
    impl->config->lineWidthSpin.setValue(value);
}


void SceneWidget::setPointSize(double value)
{
    impl->config->pointSizeSpin.setValue(value);
}


void SceneWidget::setNormalLength(double value)
{
    impl->config->normalLengthSpin.setValue(value);
}


void SceneWidget::setHeadLightEnabled(bool on)
{
    impl->config->headLightCheck.setChecked(on);
}


void SceneWidget::setHeadLightLightingFromBack(bool on)
{
    impl->config->headLightFromBackCheck.setChecked(on);
}


void SceneWidget::setWorldLight(bool on)
{
    impl->config->worldLightCheck.setChecked(on);
}


void SceneWidget::setAdditionalLights(bool on)
{
    impl->config->additionalLightsCheck.setChecked(on);
}


void SceneWidget::setFloorGrid(bool on)
{
    impl->config->gridCheck[FLOOR_GRID].setChecked(on);
}


void SceneWidget::setNormalVisualization(bool on)
{
    impl->config->normalVisualizationCheck.setChecked(on);
}


void SceneWidget::setCoordinateAxes(bool on)
{
    impl->config->coordinateAxesCheck.setChecked(on);
}


void SceneWidget::setBackgroundColor(const Vector3& color)
{
    impl->renderer->setBackgroundColor(color.cast<float>());
    impl->update();
}


Vector3 SceneWidget::backgroundColor()
{
	return impl	->renderer->backgroundColor().cast<double>();
}

void SceneWidget::setColor(const Vector4& color)
{
    impl->renderer->setColor(color.head<3>().cast<float>());
}


void SceneWidget::setShowFPS(bool on) 
{
    impl->showFPS(on);
}


void SceneWidget::setCameraPosition(const Vector3& eye, const Vector3& direction, const Vector3& up)
{
    impl->builtinCameraTransform->setPosition(SgCamera::positionLookingFor(eye, direction, up));
}


void SceneWidget::setFieldOfView(double value)
{
    impl->builtinPersCamera->setFieldOfView(value);
}


void SceneWidget::setHeight(double value)
{
    impl->builtinOrthoCamera->setHeight(value);
}


void SceneWidget::setNear(double value)
{
    impl->config->zNearSpin.setValue(value);
}


void SceneWidget::setFar(double value)
{
    impl->config->zFarSpin.setValue(value);
}


void SceneWidget::showConfigDialog()
{
    impl->config->show();
}


QVBoxLayout* SceneWidget::configDialogVBox()
{
    return impl->config->vbox;
}


bool SceneWidget::saveImage(const std::string& filename)
{
    return impl->saveImage(filename);
}


bool SceneWidgetImpl::saveImage(const std::string& filename)
{
    return grabFramebuffer().save(filename.c_str());
}


QImage SceneWidget::getImage()
{
    return impl->grabFramebuffer();
}


void SceneWidget::setScreenSize(int width, int height)
{
    impl->setScreenSize(width, height);
}


void SceneWidgetImpl::setScreenSize(int width, int height)
{
    QRect r = self->geometry();
    setGeometry((r.width() - width) / 2, (r.height() - height) / 2, width, height);
}


void SceneWidget::updateIndicator(const std::string& text)
{
    impl->indicatorLabel->setText(text.c_str());
}


void SceneWidgetImpl::updateIndicator(const std::string& text)
{
    indicatorLabel->setText(text.c_str());
}


bool SceneWidget::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool SceneWidgetImpl::storeState(Archive& archive)
{
    archive.write("editMode", isEditMode);
    archive.write("viewpointControlMode", viewpointControlMode.selectedSymbol());
    archive.write("collisionLines", collisionLinesVisible);
    archive.write("polygonMode", polygonMode.selectedSymbol());

    config->storeState(archive);

    ListingPtr cameraListing = new Listing;
    set<SgPosTransform*> storedTransforms;
    int numCameras = renderer->numCameras();
    for(int i=0; i < numCameras; ++i){
        Mapping* cameraState = 0;
        if(InteractiveCameraTransform* transform = self->findOwnerInteractiveCameraTransform(i)){
            if(!storedTransforms.insert(transform).second){
                transform = 0; // already stored
            }
            cameraState = storeCameraState(i, true, transform);
        } else {
            if(i == renderer->currentCameraIndex()){
                cameraState = storeCameraState(i, false, 0);
            }
        }
        if(cameraState){
            cameraListing->append(cameraState);
        }
    }
    if(!cameraListing->empty()){
        archive.insert("cameras", cameraListing);
    }

    write(archive, "backgroundColor", renderer->backgroundColor());
    write(archive, "gridColor", gridColor[FLOOR_GRID]);
    write(archive, "xzgridColor", gridColor[XZ_GRID]);
    write(archive, "yzgridColor", gridColor[YZ_GRID]);
    
    return true;
}


void SceneWidgetImpl::writeCameraPath(Mapping& archive, const std::string& key, int cameraIndex)
{
   vector<string> cameraStrings;
    if(renderer->getSimplifiedCameraPathStrings(cameraIndex, cameraStrings)){
        if(cameraStrings.size() == 1){
            archive.write(key, cameraStrings.front());
        } else {
            Listing& pathNode = *archive.createListing(key);
            pathNode.setFlowStyle(true);
            for(size_t i=0; i < cameraStrings.size(); ++i){
                pathNode.append(cameraStrings[i]);
            }
        }
    }
}


Mapping* SceneWidgetImpl::storeCameraState(int cameraIndex, bool isInteractiveCamera, SgPosTransform* cameraTransform)
{
    Mapping* state = new Mapping;
    writeCameraPath(*state, "camera", cameraIndex);

    if(cameraIndex == renderer->currentCameraIndex()){
        state->write("isCurrent", true);
    }

    if(isInteractiveCamera){
        SgCamera* camera = renderer->camera(cameraIndex);
        if(self->isBuiltinCamera(camera)){
            if(SgPerspectiveCamera* pers = dynamic_cast<SgPerspectiveCamera*>(camera)){
                state->write("fieldOfView", pers->fieldOfView());
            } else if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
                state->write("orthoHeight", ortho->height());
            }
            state->write("near", camera->nearClipDistance());
            state->write("far", camera->farClipDistance());
        }
        if(cameraTransform){
            const Affine3& T = cameraTransform->T();
            write(*state, "eye", T.translation());
            write(*state, "direction", SgCamera::direction(T));
            write(*state, "up", SgCamera::up(T));
        }
    }

    return state;
}


template<typename Derived> static bool readColor(const Mapping& mapping, const char* key, Eigen::MatrixBase<Derived>& out_color)
{
    const Listing& elements = *mapping.findListing(key);
    if(!elements.isValid()){
        return false;
    }
    if(elements.size() < 3 || elements.size() > 4){
        elements.throwException("The color value must have three or four elements");
    }
    for(int i=0; i < 3; ++i){
        out_color[i] = elements[i].toDouble();
    }
    if(out_color.rows() == 4){
        if(elements.size() == 4){
            out_color[3] = elements[3].toDouble();
        } else {
            out_color[3] = 1.0f;
        }
    }
    return true;
}

    
bool SceneWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool SceneWidgetImpl::restoreState(const Archive& archive)
{
    bool doUpdate = false;

    setEditMode(archive.get("editMode", isEditMode));
    
    string symbol;
    if(archive.read("viewpointControlMode", symbol)){
        self->setViewpointControlMode((SceneWidget::ViewpointControlMode(viewpointControlMode.index(symbol))));
    }
    if(archive.read("polygonMode", symbol)){
        setPolygonMode(polygonMode.index(symbol));
    }

    setCollisionLinesVisible(archive.get("collisionLines", collisionLinesVisible));
    
    config->restoreState(archive);

    const Listing& cameraListing = *archive.findListing("cameras");
    if(cameraListing.isValid()){
        archive.addPostProcess([&](){ restoreCameraStates(cameraListing); }, 1);
    } else {
        // for the compatibility to the older versions
        const Mapping& cameraData = *archive.findMapping("camera");
        if(cameraData.isValid()){
            Vector3 eye, direction, up;
            if(read(cameraData, "eye", eye) &&
               read(cameraData, "direction", direction) &&
               read(cameraData, "up", up)){
                builtinCameraTransform->setPosition(SgCamera::positionLookingFor(eye, direction, up));
            }
            double fov;
            if(cameraData.read("fieldOfView", fov)){
                builtinPersCamera->setFieldOfView(fov);
            }
            double height;
            if(cameraData.read("orthoHeight", height)){
                builtinOrthoCamera->setHeight(height);
            }
            double zNear, zFar;
            if(cameraData.read("near", zNear)){
                builtinPersCamera->setNearClipDistance(zNear);
                builtinOrthoCamera->setNearClipDistance(zNear);
            }
            if(cameraData.read("far", zFar)){
                builtinPersCamera->setFarClipDistance(zFar);
                builtinOrthoCamera->setFarClipDistance(zFar);
            }

            builtinPersCamera->notifyUpdate();
            builtinOrthoCamera->notifyUpdate();
            builtinCameraTransform->notifyUpdate();
            doUpdate = true;
            
            archive.addPostProcess([&](){ restoreCurrentCamera(cameraData); }, 1);
        }
    }

    Vector3f color;
    if(readColor(archive, "backgroundColor", color)){
        renderer->setBackgroundColor(color);
        doUpdate = true;
    }

    if(readColor(archive, "gridColor", gridColor[FLOOR_GRID])){
        doUpdate = true;
    }
    if(readColor(archive, "xzgridColor", gridColor[XZ_GRID])){
    	doUpdate = true;
    }
    if(readColor(archive, "yzgridColor", gridColor[YZ_GRID])){
    	doUpdate = true;
    }
    if(doUpdate){
        updateGrids();
        update();
    }

    return true;
}


void SceneWidgetImpl::restoreCameraStates(const Listing& cameraListing)
{
    bool doUpdate = false;

    renderer->extractPreprocessedNodes();
    
    for(int i=0; i < cameraListing.size(); ++i){
        const Mapping& state = *cameraListing[i].toMapping();
        int cameraIndex = readCameraPath(state, "camera");
        if(cameraIndex >= 0){
            Vector3 eye, direction, up;
            if(read(state, "eye", eye) &&
               read(state, "direction", direction) &&
               read(state, "up", up)){
                const SgNodePath& cameraPath = renderer->cameraPath(cameraIndex);
                for(size_t j=0; j < cameraPath.size() - 1; ++j){
                    SgPosTransform* transform = dynamic_cast<SgPosTransform*>(cameraPath[j]);
                    if(transform){
                        transform->setPosition(SgCamera::positionLookingFor(eye, direction, up));
                        transform->notifyUpdate();
                    }
                }
            }

            SgCamera* camera = renderer->camera(cameraIndex);
            if(SgPerspectiveCamera* pers = dynamic_cast<SgPerspectiveCamera*>(camera)){
                double fov;
                if(state.read("fieldOfView", fov)){
                    pers->setFieldOfView(fov);
                    doUpdate = true;
                }
            }
            if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
                double height;
                if(state.read("orthoHeight", height)){
                    ortho->setHeight(height);
                    doUpdate = true;
                }
            }
            double near, far;
            if(state.read("near", near)){
                camera->setNearClipDistance(near);
                doUpdate = true;
            }
            if(state.read("far", far)){
                camera->setFarClipDistance(far);
                doUpdate = true;
            }
            if(state.get("isCurrent", false)){
                renderer->setCurrentCamera(cameraIndex);
            }
            camera->notifyUpdate();
        }
    }

    if(doUpdate){
        update();
    }
}


int SceneWidgetImpl::readCameraPath(const Mapping& archive, const char* key)
{
    int index = -1;

    std::vector<std::string> pathStrings;
    const ValueNode& value = *archive.find(key);
    if(value.isString()){
        pathStrings.push_back(value.toString());
    } else if(value.isListing()){
        const Listing& pathNode = *value.toListing();
        for(int i=0; i < pathNode.size(); ++i){
            pathStrings.push_back(pathNode[i].toString());
        }
    }
    if(!pathStrings.empty()){
        index = renderer->findCameraPath(pathStrings);
    }

    return index;
}    


// for the compatibility to the older versions
void SceneWidgetImpl::restoreCurrentCamera(const Mapping& cameraData)
{
    renderer->extractPreprocessedNodes();
    int index = readCameraPath(cameraData, "current");
    if(index >= 0){
        renderer->setCurrentCamera(index);
    }
}


void SceneWidgetImpl::updateGrids()
{
    if(gridGroup){
        activateSystemNode(gridGroup, false);
        gridGroup = 0;
    }
        
    for(int i=0; i < 3; ++i){
        bool isActive = config->gridCheck[i].isChecked();
        if(isActive){
            if(!gridGroup){
                gridGroup = new SgInvariantGroup;
                gridGroup->setName("GridGroup");
            }
            gridGroup->addChild(createGrid(i));
        }
    }
    if(gridGroup){
        activateSystemNode(gridGroup, true);
    }

    update();
}


SgLineSet* SceneWidgetImpl::createGrid(int index)
{
    SgLineSet* grid = new SgLineSet;
    const Vector4f& c = gridColor[index];
    grid->getOrCreateMaterial()->setDiffuseColor(Vector3f(c[0], c[1], c[2]));

    SgVertexArray& vertices = *grid->getOrCreateVertices();
    
    int axis1;
    int axis2;
    if(index == 0){
        axis1 = 0;
        axis2 = 1;
    } else if(index == 1){
        axis1 = 0;
        axis2 = 2;
    } else {
        axis1 = 1;
        axis2 = 2;
    }
    Vector3f v(0.0f, 0.0f, 0.0f);
    static float sign[2] = { 1.0f, -1.0f };
    float half = config->gridSpanSpin[index].value() / 2.0f;
    float interval = config->gridIntervalSpin[index].value();
    float x = 0.0f;
    int i = 0;
    
    do {
        x = i++ * interval;
        for(int j=0; j < 2; ++j){
            for(int k=0; k < 2; ++k){
                v[axis1] = sign[j] * x;
                v[axis2] = sign[k] * half;
                vertices.push_back(v);
            }
        }
        for(int j=0; j < 2; ++j){
            for(int k=0; k < 2; ++k){
                v[axis1] = sign[k] * half;
                v[axis2] = sign[j] * x;
                vertices.push_back(v);
            }
        }
    } while(x < half);

    const int n = vertices.size();
    SgIndexArray& lineVertices = grid->lineVertices();
    lineVertices.resize(n);
    for(int i=0; i < n; ++i){
        lineVertices[i] = i;
    }

    return grid;
}


void SceneWidgetImpl::activateSystemNode(SgNode* node, bool on)
{
    if(on){
        systemGroup->addChild(node, true);
    } else {
        systemGroup->removeChild(node, true);
    }
}


ConfigDialog::ConfigDialog(SceneWidgetImpl* impl)
    : sceneWidgetImpl(impl),
      lightingMode(3, CNOID_GETTEXT_DOMAIN_NAME),
      cullingMode(GLSceneRenderer::N_CULLING_MODES, CNOID_GETTEXT_DOMAIN_NAME)
{
    setWindowTitle(_("Scene Config"));

    auto renderer = sceneWidgetImpl->renderer;

    QVBoxLayout* topVBox = new QVBoxLayout;
    vbox = new QVBoxLayout;
    QHBoxLayout* hbox;

    builtinCameraConnections.add(
        impl->builtinPersCamera->sigUpdated().connect(
            [&](const SgUpdate&){ updateBuiltinCameraConfig(); }));
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Default Camera"))));
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Field of view")));
    fieldOfViewSpin.setRange(1, 179);
    fieldOfViewSpin.sigValueChanged().connect(
        [=](int value){ impl->onFieldOfViewChanged(radian(value)); });
    hbox->addWidget(&fieldOfViewSpin);
    hbox->addWidget(new QLabel("[deg]"));
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Clipping depth")));
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Near")));
    zNearSpin.setDecimals(4);
    zNearSpin.setRange(0.0001, 9.9999);
    zNearSpin.setSingleStep(0.0001);
    zNearSpin.sigValueChanged().connect([=](double){ impl->onClippingDepthChanged(); });
    hbox->addWidget(&zNearSpin);
    hbox->addWidget(new QLabel(_("Far")));
    zFarSpin.setDecimals(1);
    zFarSpin.setRange(0.1, 9999999.9);
    zFarSpin.setSingleStep(0.1);
    zFarSpin.sigValueChanged().connect([=](double){ impl->onClippingDepthChanged(); });
    hbox->addWidget(&zFarSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    restrictCameraRollCheck.setText(_("Restrict camera roll"));
    restrictCameraRollCheck.setChecked(true);
    hbox->addWidget(&restrictCameraRollCheck);
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Vertical axis")));
    verticalAxisYRadio.setText("Y");
    hbox->addWidget(&verticalAxisYRadio);
    verticalAxisZRadio.setText("Z");
    hbox->addWidget(&verticalAxisZRadio);
    verticalAxisGroup.addButton(&verticalAxisYRadio, 0);
    verticalAxisGroup.addButton(&verticalAxisZRadio, 1);
    verticalAxisZRadio.setChecked(true);
    hbox->addStretch();
    vbox->addLayout(hbox);

    updateDefaultLightsLater.setFunction([=](){ impl->updateDefaultLights(); });
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Lighting"))));
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Lighting mode")));

    fullLightingRadio.setText(_("Full"));
    fullLightingRadio.setChecked(true);
    lightingModeGroup.addButton(&fullLightingRadio, GLSceneRenderer::FULL_LIGHTING);
    lightingMode.setSymbol(GLSceneRenderer::FULL_LIGHTING, "full");
    hbox->addWidget(&fullLightingRadio);

    normalLightingRadio.setText(_("Normal"));
    normalLightingRadio.setChecked(false);
    lightingModeGroup.addButton(&normalLightingRadio, GLSceneRenderer::NORMAL_LIGHTING);
    lightingMode.setSymbol(GLSceneRenderer::NORMAL_LIGHTING, "normal");
    hbox->addWidget(&normalLightingRadio);
    
    minLightingRadio.setText(_("Minimum"));
    lightingModeGroup.addButton(&minLightingRadio, GLSceneRenderer::MINIMUM_LIGHTING);
    lightingMode.setSymbol(GLSceneRenderer::MINIMUM_LIGHTING, "minimum");
    hbox->addWidget(&minLightingRadio);
    
    solidColorLightingRadio.setText(_("Solid"));
    lightingModeGroup.addButton(&solidColorLightingRadio, GLSceneRenderer::SOLID_COLOR_LIGHTING);
    lightingMode.setSymbol(GLSceneRenderer::SOLID_COLOR_LIGHTING, "solid");
    hbox->addWidget(&solidColorLightingRadio);

    lightingModeGroup.sigButtonToggled().connect(
        [&, renderer](int mode, bool checked){
            if(checked){
                lightingMode.select(mode);
                renderer->setLightingMode(mode);
                sceneWidgetImpl->update();
            }
        });

    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    smoothShadingCheck.setText(_("Smooth shading"));
    smoothShadingCheck.setChecked(true);
    smoothShadingCheck.sigToggled().connect([=](bool on){ impl->onSmoothShadingToggled(on); });
    hbox->addWidget(&smoothShadingCheck);

    hbox->addSpacing(10);
    hbox->addWidget(new QLabel(_("Back face culling: ")));
    cullingMode.setSymbol(GLSceneRenderer::ENABLE_BACK_FACE_CULLING, "enabled");
    cullingMode.setSymbol(GLSceneRenderer::DISABLE_BACK_FACE_CULLING, "disabled");
    cullingMode.setSymbol(GLSceneRenderer::FORCE_BACK_FACE_CULLING, "forced");
    cullingMode.select(renderer->backFaceCullingMode());
    cullingRadios[GLSceneRenderer::ENABLE_BACK_FACE_CULLING].setText(_("Enabled"));
    cullingRadios[GLSceneRenderer::DISABLE_BACK_FACE_CULLING].setText(_("Disabled"));
    cullingRadios[GLSceneRenderer::FORCE_BACK_FACE_CULLING].setText(_("Forced"));
    for(int i=0; i < cullingMode.size(); ++i){
        cullingModeGroup.addButton(&cullingRadios[i], i);
        hbox->addWidget(&cullingRadios[i]);
    }
    cullingRadios[cullingMode.which()].setChecked(true);
    cullingModeGroup.sigButtonToggled().connect(
        [&, renderer](int mode, bool checked){
            if(checked){
                cullingMode.select(mode);
                renderer->setBackFaceCullingMode(mode);
                sceneWidgetImpl->update();
            }
        });

    hbox->addStretch();
    vbox->addLayout(hbox);

    auto grid = new QGridLayout;
    
    headLightCheck.setText(_("Head light"));
    headLightCheck.setChecked(true);
    headLightCheck.sigToggled().connect([&](bool){ updateDefaultLightsLater(); });
    grid->addWidget(&headLightCheck, 0, 0);

    grid->addWidget(new QLabel(_("Intensity")), 0, 1);
    headLightIntensitySpin.setDecimals(2);
    headLightIntensitySpin.setSingleStep(0.01);    
    headLightIntensitySpin.setRange(0.0, 2.0);
    headLightIntensitySpin.setValue(0.75);
    headLightIntensitySpin.sigValueChanged().connect([&](bool){ updateDefaultLightsLater(); });
    grid->addWidget(&headLightIntensitySpin, 0, 2);

    headLightFromBackCheck.setText(_("Back lighting"));
    headLightFromBackCheck.sigToggled().connect([&](bool){ updateDefaultLightsLater(); });
    grid->addWidget(&headLightFromBackCheck, 0, 3, 1, 2);

    worldLightCheck.setText(_("World light"));
    worldLightCheck.setChecked(true);
    worldLightCheck.sigToggled().connect([&](bool){ updateDefaultLightsLater(); });
    grid->addWidget(&worldLightCheck, 1, 0);

    grid->addWidget(new QLabel(_("Intensity")), 1, 1);
    worldLightIntensitySpin.setDecimals(2);
    worldLightIntensitySpin.setSingleStep(0.01);    
    worldLightIntensitySpin.setRange(0.0, 2.0);
    worldLightIntensitySpin.setValue(0.5);
    worldLightIntensitySpin.sigValueChanged().connect([&](double){ updateDefaultLightsLater(); });
    grid->addWidget(&worldLightIntensitySpin, 1, 2);

    grid->addWidget(new QLabel(_("Ambient")), 1, 3);
    worldLightAmbientSpin.setDecimals(2);
    worldLightAmbientSpin.setSingleStep(0.01);    
    worldLightAmbientSpin.setRange(0.0, 1.0);
    worldLightAmbientSpin.setValue(0.25);
    worldLightAmbientSpin.sigValueChanged().connect([&](double){ updateDefaultLightsLater(); });
    grid->addWidget(&worldLightAmbientSpin, 1, 4);

    grid->setColumnStretch(5, 1);
    additionalLightsCheck.setText(_("Additional lights"));
    additionalLightsCheck.setChecked(true);
    additionalLightsCheck.sigToggled().connect([&](bool){ updateDefaultLightsLater(); });
    grid->addWidget(&additionalLightsCheck, 1, 6);

    grid->setColumnStretch(7, 10);
    vbox->addLayout(grid);

    hbox = new QHBoxLayout;
    textureCheck.setText(_("Texture"));
    textureCheck.setChecked(true);
    textureCheck.sigToggled().connect([=](bool on){ impl->onTextureToggled(on); });
    hbox->addWidget(&textureCheck);

    fogCheck.setText(_("Fog"));
    fogCheck.setChecked(true);
    fogCheck.sigToggled().connect([&](bool){ updateDefaultLightsLater(); });
    hbox->addWidget(&fogCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    for(int i=0; i < NUM_SHADOWS; ++i){
        Shadow& shadow = shadows[i];
        shadow.check.setText(QString(_("Shadow %1")).arg(i+1));
        shadow.check.setChecked(false);
        shadow.check.sigToggled().connect([&](bool){ updateDefaultLightsLater(); });
        hbox->addWidget(&shadow.check);
        shadow.lightLabel.setText(_("Light"));
        hbox->addWidget(&shadow.lightLabel);
        shadow.lightSpin.setRange(0, 99);
        shadow.lightSpin.setValue(0);
        shadow.lightSpin.sigValueChanged().connect([&](double){ updateDefaultLightsLater(); });
        hbox->addWidget(&shadow.lightSpin);
    }

    shadowAntiAliasingCheck.setText(_("Anti-aliasing of shadows"));
    shadowAntiAliasingCheck.setChecked(true);
    shadowAntiAliasingCheck.sigToggled().connect([&](bool){ updateDefaultLightsLater(); });
    hbox->addWidget(&shadowAntiAliasingCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addLayout(new HSeparatorBox(new QLabel(_("Background"))));

    grid = new QGridLayout;
    backgroundColorButton.setText(_("Background color"));
    backgroundColorButton.sigClicked().connect([=](){ impl->showBackgroundColorDialog(); });
    grid->addWidget(&backgroundColorButton, 0, 0);
    
    coordinateAxesCheck.setText(_("Coordinate axes"));
    coordinateAxesCheck.setChecked(true);
    coordinateAxesCheck.sigToggled().connect(
        [=](bool on){ impl->activateSystemNode(impl->coordinateAxesOverlay, on); });
    grid->addWidget(&coordinateAxesCheck, 2, 0);

    grid->addWidget(new VSeparator, 0, 2, 3, 1);
    grid->setColumnStretch(1, 1);
    grid->setColumnStretch(3, 1);

    gridCheck[FLOOR_GRID].setText(_("XY Grid"));
    gridCheck[FLOOR_GRID].setChecked(true);
    gridCheck[XZ_GRID].setText(_("XZ Grid"));
    gridCheck[YZ_GRID].setText(_("YZ Grid"));
    
    for(int i=0; i < 3; ++i){
        auto& check = gridCheck[i];
    	check.sigToggled().connect([=](bool){ impl->updateGridsLater(); });
        grid->addWidget(&check, i, 4);

        grid->addWidget(new QLabel(_("Span")), i, 5);
        auto& spanSpin = gridSpanSpin[i];
        spanSpin.setAlignment(Qt::AlignCenter);
    	spanSpin.setDecimals(1);
    	spanSpin.setRange(0.0, 99.9);
        spanSpin.setSingleStep(0.1);
        spanSpin.setValue(10.0);
        spanSpin.sigValueChanged().connect([=](double){ impl->updateGridsLater(); });
        grid->addWidget(&spanSpin, i, 6);
        
        grid->addWidget(new QLabel(_("Interval")), i, 7);
        auto& intervalSpin = gridIntervalSpin[i];
        intervalSpin.setAlignment(Qt::AlignCenter);
        intervalSpin.setDecimals(2);
        intervalSpin.setRange(0.01, 9.99);
        intervalSpin.setSingleStep(0.01);
        intervalSpin.setValue(0.5);
        intervalSpin.sigValueChanged().connect([=](double){ impl->updateGridsLater(); });
        grid->addWidget(&intervalSpin, i, 8);

        auto button = new PushButton;
        button->setText(_("Color"));
        button->sigClicked().connect([=](){ impl->showGridColorDialog(i); });
        grid->addWidget(button, i, 9);
    }

    grid->setColumnStretch(10, 10);
    vbox->addLayout(grid);

    vbox->addWidget(new HSeparator);
    
    hbox = new QHBoxLayout;
    defaultColorButton.setText(_("Default color"));
    defaultColorButton.sigClicked().connect([=](){ impl->showDefaultColorDialog(); });
    hbox->addWidget(&defaultColorButton);
    
    hbox->addWidget(new QLabel(_("Default line width")));
    lineWidthSpin.setDecimals(1);
    lineWidthSpin.setRange(0.1, 9.9);
    lineWidthSpin.setSingleStep(0.1);
    lineWidthSpin.setValue(1.0);
    lineWidthSpin.sigValueChanged().connect([=](double width){ impl->onLineWidthChanged(width); });
    hbox->addWidget(&lineWidthSpin);

    hbox->addWidget(new QLabel(_("Default point size")));
    pointSizeSpin.setDecimals(1);
    pointSizeSpin.setRange(0.1, 9.9);
    pointSizeSpin.setSingleStep(0.1);
    pointSizeSpin.setValue(1.0);
    pointSizeSpin.sigValueChanged().connect([=](double size){ impl->onPointSizeChanged(size); });
    hbox->addWidget(&pointSizeSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    pointRenderingModeCheck.setText(_("Do point rendering in the wireframe mode"));
    pointRenderingModeCheckConnection =
        pointRenderingModeCheck.sigToggled().connect([=](bool on){ impl->onPointRenderingModeToggled(on); });
    hbox->addWidget(&pointRenderingModeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout;
    normalVisualizationCheck.setText(_("Normal Visualization"));
    normalVisualizationCheck.sigToggled().connect([=](bool){ impl->onNormalVisualizationChanged(); });
    hbox->addWidget(&normalVisualizationCheck);
    normalLengthSpin.setDecimals(3);
    normalLengthSpin.setRange(0.0, 1000.0);
    normalLengthSpin.setSingleStep(0.001);
    normalLengthSpin.setValue(0.01);
    normalLengthSpin.sigValueChanged().connect([=](double){ impl->onNormalVisualizationChanged(); });
    hbox->addWidget(&normalLengthSpin);

    lightweightViewChangeCheck.setText(_("Lightweight view change"));
    lightweightViewChangeCheck.sigToggled().connect(
        [=](bool on){ impl->isLightweightViewChangeEnabled = on; });
    hbox->addWidget(&lightweightViewChangeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    vbox->addWidget(new HSeparator);

    hbox = new QHBoxLayout;

    /*
    fpsCheck.setText(_("Show FPS"));
    fpsCheck.setChecked(false);
    fpsCheck.sigToggled().connect([=](bool on){ impl->showFPS(on); });
    hbox->addWidget(&fpsCheck);
    */

    fpsTestButton.setText(_("FPS Test"));
    fpsTestButton.sigClicked().connect([=](){ impl->onFPSTestButtonClicked(); });
    hbox->addWidget(&fpsTestButton);
    fpsTestIterationSpin.setRange(1, 99);
    fpsTestIterationSpin.setValue(1);
    hbox->addWidget(&fpsTestIterationSpin);

    auto pickingBufferButton = new PushButton(_("Show the picking buffer image (for debug)"));
    pickingBufferButton->sigClicked().connect([=](){ impl->showPickingBufferImageWindow(); });
    hbox->addWidget(pickingBufferButton);
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout;
    collisionVisualizationButtonsCheck.setText(_("Show collision visualization button set"));
    collisionVisualizationButtonsCheck.setChecked(false);
    collisionVisualizationButtonsCheck.sigToggled().connect(
        [&](bool on){
            auto sceneBar = SceneBar::instance();
            sceneBar->setCollisionVisualizationButtonSetVisible(on);
            sceneBar->toolBarArea()->layoutToolBars();
        });
    hbox->addWidget(&collisionVisualizationButtonsCheck);
    
    upsideDownCheck.setText(_("Upside down"));
    upsideDownCheck.setChecked(false);
    upsideDownCheck.sigToggled().connect([=](bool on){ impl->onUpsideDownToggled(on); });
    hbox->addWidget(&upsideDownCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    topVBox->addLayout(vbox);
    topVBox->addWidget(new HSeparator);

    QPushButton* okButton = new QPushButton(_("&Ok"));
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    topVBox->addWidget(buttonBox);
    
    setLayout(topVBox);
}


void ConfigDialog::showEvent(QShowEvent* event)
{
    if(!sceneWidgetImpl->renderer->isShadowCastingAvailable()){
        for(int i=0; i < NUM_SHADOWS; ++i){
            auto& shadow = shadows[i];
            auto& check = shadow.check;
            check.setEnabled(false);
            check.setChecked(false);
            shadow.lightLabel.setEnabled(false);
            shadow.lightSpin.setEnabled(false);
        }
        shadowAntiAliasingCheck.setEnabled(false);
    }
}
            
    
void ConfigDialog::updateBuiltinCameraConfig()
{
    auto persCamera = sceneWidgetImpl->builtinPersCamera;

    fieldOfViewSpin.blockSignals(true);
    fieldOfViewSpin.setValue(round(degree(persCamera->fieldOfView())));
    fieldOfViewSpin.blockSignals(false);
    
    zNearSpin.blockSignals(true);
    zNearSpin.setValue(persCamera->nearClipDistance());
    zNearSpin.blockSignals(false);

    zFarSpin.blockSignals(true);
    zFarSpin.setValue(persCamera->farClipDistance());
    zFarSpin.blockSignals(false);
}


void ConfigDialog::storeState(Archive& archive)
{
    archive.write("restrictCameraRoll", restrictCameraRollCheck.isChecked());
    archive.write("verticalAxis", verticalAxisZRadio.isChecked() ? "Z" : "Y");
    archive.write("lightingMode", lightingMode.selectedSymbol());
    archive.write("cullingMode", cullingMode.selectedSymbol());
    archive.write("defaultHeadLight", headLightCheck.isChecked());
    archive.write("defaultHeadLightIntensity", headLightIntensitySpin.value());
    archive.write("headLightLightingFromBack", headLightFromBackCheck.isChecked());
    archive.write("worldLight", worldLightCheck.isChecked());
    archive.write("worldLightIntensity", worldLightIntensitySpin.value());
    archive.write("worldLightAmbient", worldLightAmbientSpin.value());
    archive.write("additionalLights", additionalLightsCheck.isChecked());

    ListingPtr shadowLights = new Listing;
    for(int i=0; i < NUM_SHADOWS; ++i){
        if(shadows[i].check.isChecked()){
            shadowLights->append(shadows[i].lightSpin.value());
        }
    }
    if(!shadowLights->empty()){
        archive.insert("shadowLights", shadowLights);
    }
    
    archive.write("fog", fogCheck.isChecked());
    archive.write("floorGrid", gridCheck[FLOOR_GRID].isChecked());
    archive.write("floorGridSpan", gridSpanSpin[FLOOR_GRID].value());
    archive.write("floorGridInterval", gridIntervalSpin[FLOOR_GRID].value());
    archive.write("xzGrid", gridCheck[XZ_GRID].isChecked());
    archive.write("xzGridSpan", gridSpanSpin[XZ_GRID].value());
    archive.write("xzGridInterval", gridIntervalSpin[YZ_GRID].value());
    archive.write("xzGrid", gridCheck[YZ_GRID].isChecked());
    archive.write("yzGridSpan", gridSpanSpin[YZ_GRID].value());
    archive.write("yzGridInterval", gridIntervalSpin[YZ_GRID].value());
    archive.write("texture", textureCheck.isChecked());
    archive.write("lineWidth", lineWidthSpin.value());
    archive.write("pointSize", pointSizeSpin.value());
    archive.write("normalVisualization", normalVisualizationCheck.isChecked());
    archive.write("normalLength", normalLengthSpin.value());
    archive.write("lightweightViewChange", lightweightViewChangeCheck.isChecked());
    archive.write("coordinateAxes", coordinateAxesCheck.isChecked());
    archive.write("fpsTestIteration", fpsTestIterationSpin.value());
    //archive.write("showFPS", fpsCheck.isChecked());
    archive.write("upsideDown", upsideDownCheck.isChecked());
}


void ConfigDialog::restoreState(const Archive& archive)
{
    string symbol;
    
    restrictCameraRollCheck.setChecked(archive.get("restrictCameraRoll", restrictCameraRollCheck.isChecked()));
    if(archive.read("verticalAxis", symbol)){
        if(symbol == "Z"){
            verticalAxisZRadio.setChecked(true);
        } else if(symbol == "Y"){
            verticalAxisYRadio.setChecked(true);
        }
    }
    
    if(archive.read("lightingMode", symbol)){
        if(lightingMode.select(symbol)){
            lightingModeGroup.button(lightingMode.which())->setChecked(true);
        }
    }
    if(archive.read("cullingMode", symbol)){
        if(cullingMode.select(symbol)){
            cullingRadios[cullingMode.which()].setChecked(true);
        }
    }
    
    headLightCheck.setChecked(archive.get("defaultHeadLight", headLightCheck.isChecked()));
    headLightIntensitySpin.setValue(archive.get("defaultHeadLightIntensity", headLightIntensitySpin.value()));
    headLightFromBackCheck.setChecked(archive.get("headLightLightingFromBack", headLightFromBackCheck.isChecked()));
    worldLightCheck.setChecked(archive.get("worldLight", worldLightCheck.isChecked()));
    worldLightIntensitySpin.setValue(archive.get("worldLightIntensity", worldLightIntensitySpin.value()));
    worldLightAmbientSpin.setValue(archive.get("worldLightAmbient", worldLightAmbientSpin.value()));
    additionalLightsCheck.setChecked(archive.get("additionalLights", additionalLightsCheck.isChecked()));

    for(int i=0; i < NUM_SHADOWS; ++i){
        shadows[i].check.setChecked(false);
    }
    
    Listing& shadowLights = *archive.findListing("shadowLights");
    if(shadowLights.isValid()){
        int configIndex = 0;
        for(int i=0; i < shadowLights.size(); ++i){
            if(configIndex == NUM_SHADOWS){
                break;
            }
            Shadow& shadow = shadows[configIndex++];
            shadow.lightSpin.setValue(shadowLights[i].toInt());
            shadow.check.setChecked(true);
        }
    }

    fogCheck.setChecked(archive.get("fog", fogCheck.isChecked()));
    gridCheck[FLOOR_GRID].setChecked(archive.get("floorGrid", gridCheck[FLOOR_GRID].isChecked()));
    gridSpanSpin[FLOOR_GRID].setValue(archive.get("floorGridSpan", gridSpanSpin[FLOOR_GRID].value()));
    gridIntervalSpin[FLOOR_GRID].setValue(archive.get("floorGridInterval", gridIntervalSpin[FLOOR_GRID].value()));
    gridCheck[XZ_GRID].setChecked(archive.get("xzGrid", gridCheck[XZ_GRID].isChecked()));
    gridSpanSpin[XZ_GRID].setValue(archive.get("xzGridSpan", gridSpanSpin[XZ_GRID].value()));
    gridIntervalSpin[XZ_GRID].setValue(archive.get("xzGridInterval", gridIntervalSpin[XZ_GRID].value()));
    gridCheck[YZ_GRID].setChecked(archive.get("yzGrid", gridCheck[YZ_GRID].isChecked()));
    gridSpanSpin[YZ_GRID].setValue(archive.get("yzGridSpan", gridSpanSpin[YZ_GRID].value()));
    gridIntervalSpin[YZ_GRID].setValue(archive.get("yzGridInterval", gridIntervalSpin[YZ_GRID].value()));
    textureCheck.setChecked(archive.get("texture", textureCheck.isChecked()));
    lineWidthSpin.setValue(archive.get("lineWidth", lineWidthSpin.value()));
    pointSizeSpin.setValue(archive.get("pointSize", pointSizeSpin.value()));
    normalVisualizationCheck.setChecked(archive.get("normalVisualization", normalVisualizationCheck.isChecked()));
    normalLengthSpin.setValue(archive.get("normalLength", normalLengthSpin.value()));
    coordinateAxesCheck.setChecked(archive.get("coordinateAxes", coordinateAxesCheck.isChecked()));
    lightweightViewChangeCheck.setChecked(archive.get("lightweightViewChange", lightweightViewChangeCheck.isChecked()));

    fpsTestIterationSpin.setValue(archive.get("fpsTestIteration", fpsTestIterationSpin.value()));
    //fpsCheck.setChecked(archive.get("showFPS", fpsCheck.isChecked()));
    upsideDownCheck.setChecked(archive.get("upsideDown", upsideDownCheck.isChecked()));
}
