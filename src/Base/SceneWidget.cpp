/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidget.h"
#include "SceneBar.h"
#include "ToolBarArea.h"
#include "GL1SceneRenderer.h"
#include "GLSLSceneRenderer.h"
#include "SceneWidgetEventHandler.h"
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
#include <cnoid/SceneEffects>
#include <cnoid/CoordinateAxesOverlay>
#include <cnoid/ConnectionSet>
#include <QOpenGLWidget>
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <QLabel>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QDialogButtonBox>
#include <QColorDialog>
#include <QElapsedTimer>
#include <QMessageBox>
#include <QCoreApplication>
#include <QGridLayout>
#include <QPainter>
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

const int ThreashToStartPointerMoveEventForEditableNode = 0;

const int NUM_SHADOWS = 2;

enum { FLOOR_GRID = 0, XZ_GRID = 1, YZ_GRID = 2 };

bool isLowMemoryConsumptionMode;
Signal<void(bool on)> sigLowMemoryConsumptionModeChanged;

QLabel* sharedIndicatorLabel = nullptr;

Signal<void(SceneWidget* instance)> sigSceneWidgetCreated;
Signal<void(SceneWidget* requester)> sigModeSyncRequest;
bool isEditModeInModeSync = false;
bool isHighlightingEnabledInModeSync = false;

class ConfigDialog : public Dialog
{
public:
    SceneWidget::Impl* sceneWidgetImpl;
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
    CheckBox normalVisualizationCheck;
    DoubleSpinBox normalLengthSpin;
    CheckBox lightweightViewChangeCheck;
    //CheckBox fpsCheck;
    PushButton fpsTestButton;
    SpinBox fpsTestIterationSpin;
    CheckBox upsideDownCheck;

    LazyCaller updateDefaultLightsLater;

    ConfigDialog(SceneWidget::Impl* impl);
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
    void setImage(const Image& image, int cursorX, int cursorY){
        if(image.numComponents() == 4){
            pixmap.convertFromImage(
                QImage(image.pixels(), image.width(), image.height(), QImage::Format_RGBA8888));
            pixmap.setDevicePixelRatio(devicePixelRatio());

            QPainter painter(&pixmap);
            painter.setPen(Qt::white);
            painter.drawLine(cursorX - 4, cursorY, cursorX + 4, cursorY);
            painter.drawLine(cursorX, cursorY - 4, cursorX, cursorY + 4);
            
            label.setPixmap(pixmap);
        }
    }
};


struct EditableNodeInfo
{
    SgNodePtr node;
    SceneWidgetEventHandler* handler;
    EditableNodeInfo() : handler(nullptr) { }
    EditableNodeInfo(SgNode* node, SceneWidgetEventHandler* handler)
        : node(node), handler(handler) { }
    bool operator<(const EditableNodeInfo& rhs) const { return handler < rhs.handler; };
    bool operator==(const EditableNodeInfo& rhs) const { return handler == rhs.handler; }
    bool operator!=(const EditableNodeInfo& rhs) const { return handler != rhs.handler; }
    explicit operator bool() const { return handler != nullptr; }
    void clear(){ node.reset(); handler = nullptr; }
};

            
}

namespace cnoid {

class SceneWidget::Impl : public QOpenGLWidget
{
    friend class SceneWidget;
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneWidget* self;

    SceneWidgetRootPtr sceneRoot;
    SgUnpickableGroupPtr systemGroup;
    SgGroupPtr scene;
    SgPolygonDrawStylePtr polygonDrawStyle;
    GLSceneRenderer* renderer;
    GLSLSceneRenderer* glslRenderer;
    GLuint prevDefaultFramebufferObject;
    bool isRendering;
    bool needToUpdatePreprocessedNodeTree;
    bool needToClearGLOnFrameBufferChange;
    SgUpdate sgUpdate;

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

    bool needToUpdateViewportInformation;
    bool isEditMode;
    bool isHighlightingEnabled;
    bool isModeSyncEnabled;
    ScopedConnection modeSyncConnection;

    Selection viewpointOperationMode;
    bool isFirstPersonMode() const {
        return (viewpointOperationMode.which() != ThirdPersonMode); }
        
    enum DragMode { NO_DRAGGING, ABOUT_TO_EDIT, EDITING, VIEW_ROTATION, VIEW_TRANSLATION, VIEW_ZOOM } dragMode;

    bool isDraggingView() const {
        return (dragMode == VIEW_ROTATION ||
                dragMode == VIEW_TRANSLATION ||
                dragMode == VIEW_ZOOM);
    }

    typedef list<vector<EditableNodeInfo>> EditableArrayList;
    EditableArrayList tmpEditableArrays;
    EditableArrayList::iterator pCurrentTmpEditableArray;
    vector<EditableNodeInfo> pointedEditablePath;
    EditableNodeInfo lastMouseMovedEditable;
    EditableNodeInfo focusedEditable;
    vector<EditableNodeInfo> focusedEditablePath;

    QCursor defaultCursor;
    QCursor editModeCursor;

    double orgMouseX;
    double orgMouseY;
    Vector3 orgPointedPos;
    Isometry3 orgCameraPosition;
    double orgOrthoCameraHeight;
    Vector3 cameraViewChangeCenter;
        
    double dragAngleRatio;
    double viewTranslationRatioX;
    double viewTranslationRatioY;

    SceneWidgetEvent latestEvent;
    QPoint latestGlobalMousePos;
    Vector3 lastClickedPoint;
    int mousePressX;
    int mousePressY;
    QMouseEvent* lastMouseMoveEvent;

    SceneWidgetEventHandler* activeCustomModeHandler;
    int activeCustomModeId;

    bool collisionLineVisibility;

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

    MenuManager menuManager;
    Signal<void(SceneWidgetEvent* event, MenuManager* menuManager)> sigContextMenuRequest;

    Signal<void(bool isFocused)> sigWidgetFocusChanged;
    Signal<void()> sigAboutToBeDestroyed;

    ImageWindow* pickingImageWindow;

    static void onOpenGLVSyncToggled(bool on, bool doConfigOutput);
    static void onLowMemoryConsumptionModeChanged(bool on, bool doConfigOutput);

    Impl(SceneWidget* self);
    ~Impl();

    void onModeSyncRequest(SceneWidget* requester);
    void onLowMemoryConsumptionModeChanged(bool on);
    void tryToResumeNormalRendering();
    void updateGrids();
    SgLineSet* createGrid(int index);

    void onSceneGraphUpdated(const SgUpdate& update);
    void extractEditablesInSubTree(SgNode* node, vector<EditableNodeInfo>& editables);
    void checkAddedEditableNodes(SgNode* node);
    void checkRemovedEditableNodes(SgNode* node);
    void warnRecursiveEditableNodeSetChange();
    
    virtual void initializeGL() override;
    virtual void resizeGL(int width, int height) override;
    virtual void paintGL() override;

    void showFPS(bool on);
    void onFPSTestButtonClicked();
    void doFPSTest();
    void onFPSUpdateRequest();
    void onFPSRenderingRequest();
    void renderFPS();

    void showBackgroundColorDialog();
    void showGridColorDialog(int index);
    void showDefaultColorDialog();

    void onCurrentCameraChanged();

    void onTextureToggled(bool on);
    void onLineWidthChanged(double width);
    void onPointSizeChanged(double width);
    void setVisiblePolygonElements(int elementFlags);
    int visiblePolygonElements() const;
    void setCollisionLineVisibility(bool on);
    void onFieldOfViewChanged(double fov);
    void onClippingDepthChanged();
    void onSmoothShadingToggled(bool on);
    void updateDefaultLights();
    void onNormalVisualizationChanged();

    void resetCursor();
    void setEditMode(bool on, bool doAdvertise);
    void toggleEditMode();
    void advertiseSceneModeChange(bool doModeSyncRequest);
    void advertiseSceneModeChangeInSubTree(SgNode* node);
    void viewAll();

    void showPickingImageWindow();
    void onUpsideDownToggled(bool on);
        
    void updateLatestEvent(QKeyEvent* event);
    void updateLatestEvent(int x, int y, int modifiers);
    void updateLatestEvent(QMouseEvent* event);
    void updateLatestEventPath(bool forceFullPicking = false);
    void updateLastClickedPoint();
        
    EditableNodeInfo applyEditableFunction(
        vector<EditableNodeInfo>& editablePath, std::function<bool(SceneWidgetEventHandler* editable)> function);
    bool setFocusToEditablePath(vector<EditableNodeInfo>& editablePath);
    bool setFocusToPointedEditablePath(const EditableNodeInfo& targetEditable);

    virtual void keyPressEvent(QKeyEvent* event) override;
    virtual void keyReleaseEvent(QKeyEvent* event) override;
    virtual void mousePressEvent(QMouseEvent* event) override;
    virtual void mouseDoubleClickEvent(QMouseEvent* event) override;
    virtual void mouseMoveEvent(QMouseEvent* event) override;
    virtual void leaveEvent(QEvent* event) override;
    string findObjectNameFromNodePath(const SgNodePath& path);
    void findObjectNameFromChildren(SgObject* object, string& name);
    virtual void mouseReleaseEvent(QMouseEvent* event) override;
    virtual void wheelEvent(QWheelEvent* event) override;
    virtual void focusInEvent(QFocusEvent* event) override;
    virtual void focusOutEvent(QFocusEvent* event) override;

    Isometry3 getNormalizedCameraTransform(const Isometry3& T);
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

    void setScreenSize(int width, int height);
    void updateIndicator(const std::string& text);
    bool storeState(Archive& archive);
    void writeCameraPath(Mapping& archive, const std::string& key, int cameraIndex);
    Mapping* storeCameraState(int cameraIndex, bool isInteractiveCamera, SgPosTransform* cameraTransform);
    bool restoreState(const Archive& archive);
    bool restoreCameraStates(const Listing& cameraListing, bool isSecondTrial);
    int readCameraPath(const Mapping& archive, const char* key);
    void restoreCurrentCamera(const Mapping& cameraData);
};

}


void SceneWidget::initializeClass(ExtensionManager* ext)
{
    Mapping* glConfig = AppConfig::archive()->openMapping("open_gl");
    
    bool isVSyncEnabled = (glConfig->get("vsync", 0) > 0);
    auto& mm = ext->menuManager();
    auto vsyncItem = mm.setPath("/Options/OpenGL").addCheckItem(_("Vertical sync"));
    vsyncItem->setChecked(isVSyncEnabled);
    vsyncItem->sigToggled().connect([&](bool on){ Impl::onOpenGLVSyncToggled(on, true); });

    isLowMemoryConsumptionMode = glConfig->get("lowMemoryConsumption", false);
    auto memoryItem = mm.addCheckItem(_("Low GPU memory consumption mode"));
    memoryItem->setChecked(isLowMemoryConsumptionMode);
    memoryItem->sigToggled().connect([&](bool on){ Impl::onLowMemoryConsumptionModeChanged(on, true); });
    Impl::onLowMemoryConsumptionModeChanged(isLowMemoryConsumptionMode, false);
}


void SceneWidget::Impl::onOpenGLVSyncToggled(bool on, bool doConfigOutput)
{
    /*
      When the menu check item on the OpenGL vertical sync is toggled, the state is
      just saved into the config file and the application must be restarted to
      update the vsync state because it is impossible to change the state of the existing
      QOpenGLWidgets.
    */
    if(doConfigOutput){
        Mapping* glConfig = AppConfig::archive()->openMapping("open_gl");
        glConfig->write("vsync", (on ? 1 : 0));
    }
}


void SceneWidget::Impl::onLowMemoryConsumptionModeChanged(bool on, bool doConfigOutput)
{
    sigLowMemoryConsumptionModeChanged(on);

    if(doConfigOutput){
        Mapping* glConfig = AppConfig::archive()->openMapping("open_gl");
        glConfig->write("low_memory_consumption", on);
    }
}


SceneWidgetRoot::SceneWidgetRoot(SceneWidget* sceneWidget)
    : sceneWidget_(sceneWidget)
{
    systemGroup = new SgUnpickableGroup;
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


SceneWidget::SceneWidget(QWidget* parent)
    : Widget(parent)
{
    impl = new Impl(this);

    ::sigSceneWidgetCreated(this);
}


SceneWidget::Impl::Impl(SceneWidget* self)
    : QOpenGLWidget(self),
      self(self),
      sceneRoot(new SceneWidgetRoot(self)),
      systemGroup(sceneRoot->systemGroup),
      emitSigStateChangedLater(std::ref(sigStateChanged)),
      updateGridsLater([this](){ updateGrids(); })
{
    setFocusPolicy(Qt::WheelFocus);
    
    auto vbox = new QVBoxLayout;
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->addWidget(this);
    self->setLayout(vbox);

    renderer = GLSceneRenderer::create(sceneRoot);
    glslRenderer = dynamic_cast<GLSLSceneRenderer*>(renderer);
    if(glslRenderer ){
        glslRenderer->setLowMemoryConsumptionMode(isLowMemoryConsumptionMode);
    }
        
    renderer->setOutputStream(MessageView::instance()->cout(false));
    renderer->enableUnusedResourceCheck(true);
    renderer->sigCurrentCameraChanged().connect([&](){ onCurrentCameraChanged(); });
    renderer->setCurrentCameraAutoRestorationMode(true);
    self->sigObjectNameChanged().connect([this](string name){ renderer->setName(name); });

    sceneRoot->sigUpdated().connect([this](const SgUpdate& update){ onSceneGraphUpdated(update); });
    tmpEditableArrays.emplace_back();
    pCurrentTmpEditableArray = tmpEditableArrays.begin();

    scene = renderer->scene();
    prevDefaultFramebufferObject = 0;
    isRendering = false;

    needToUpdatePreprocessedNodeTree = true;
    renderer->setFlagVariableToUpdatePreprocessedNodeTree(needToUpdatePreprocessedNodeTree);

    needToClearGLOnFrameBufferChange = false;

    for(auto& color : gridColor){
    	color << 0.9f, 0.9f, 0.9f, 1.0f;
    }

    setAutoFillBackground(false);
    setMouseTracking(true);
    
    needToUpdateViewportInformation = true;
    isEditMode = false;
    isHighlightingEnabled = false;
    isModeSyncEnabled = false;
    viewpointOperationMode.resize(2);
    viewpointOperationMode.setSymbol(ThirdPersonMode, "thirdPerson");
    viewpointOperationMode.setSymbol(FirstPersonMode, "firstPerson");
    viewpointOperationMode.select(ThirdPersonMode);
    dragMode = NO_DRAGGING;
    defaultCursor = self->cursor();
    editModeCursor = QCursor(Qt::PointingHandCursor);

    latestEvent.sceneWidget_ = self;
    lastClickedPoint.setZero();
    lastMouseMoveEvent = nullptr;

    activeCustomModeHandler = nullptr;
    activeCustomModeId = 0;

    if(!sharedIndicatorLabel){
        sharedIndicatorLabel = new QLabel;
        sharedIndicatorLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
        QFont font = sharedIndicatorLabel->font();
        font.setFixedPitch(true);
        sharedIndicatorLabel->setFont(font);
    }

    builtinCameraTransform = new InteractiveCameraTransform;
    builtinCameraTransform->setTransform(
        SgCamera::positionLookingAt(
            Vector3(3.0, 1.5, 1.2), Vector3(0, 0, 0.6), Vector3::UnitZ()));
    interactiveCameraTransform = builtinCameraTransform;

    builtinPersCamera = new SgPerspectiveCamera;
    builtinPersCamera->setName("Perspective");
    builtinPersCamera->setFieldOfView(radian(35.0));
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

    collisionLineVisibility = false;

    coordinateAxesOverlay = new CoordinateAxesOverlay;
    activateSystemNode(coordinateAxesOverlay, config->coordinateAxesCheck.isChecked());

    updateGrids();

    /*
    fpsTimer.sigTimeout().connect([&](){ onFPSUpdateRequest(); });
    fpsRenderingTimer.setSingleShot(true);
    fpsRenderingTimer.sigTimeout().connect([&](){ onFPSRenderingRequest(); });
    */

    isDoingFPSTest = false;

    sigLowMemoryConsumptionModeChanged.connect(
        [&](bool on){ onLowMemoryConsumptionModeChanged(on); });

    pickingImageWindow = nullptr;
}


SceneWidget::~SceneWidget()
{
    impl->sigAboutToBeDestroyed();
    delete impl;
}


SceneWidget::Impl::~Impl()
{
    delete renderer;
    delete config;

    if(lastMouseMoveEvent){
        delete lastMouseMoveEvent;
    }
    if(pickingImageWindow){
        delete pickingImageWindow;
    }
}


void SceneWidget::setModeSyncEnabled(bool on)
{
    if(on != impl->isModeSyncEnabled){
        impl->isModeSyncEnabled = on;
        if(on){
            impl->modeSyncConnection =
                sigModeSyncRequest.connect(
                    [this](SceneWidget* requester){ impl->onModeSyncRequest(requester); });
            impl->isEditMode = isEditModeInModeSync;
            impl->isHighlightingEnabled = isHighlightingEnabledInModeSync;
        } else {
            impl->modeSyncConnection.disconnect();
        }
    }
}


void SceneWidget::Impl::onModeSyncRequest(SceneWidget* requester)
{
    isHighlightingEnabled = requester->isHighlightingEnabled();
    setEditMode(requester->isEditMode(), false);
    advertiseSceneModeChange(false);
}


void SceneWidget::Impl::onLowMemoryConsumptionModeChanged(bool on)
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


SgGroup* SceneWidget::systemNodeGroup()
{
    return impl->systemGroup;
}


SceneRenderer* SceneWidget::renderer()
{
    return impl->renderer;
}


void SceneWidget::renderScene(bool doImmediately)
{
    if(doImmediately){
        impl->repaint();
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents|QEventLoop::ExcludeSocketNotifiers);
    } else {
        impl->update();
    }
}


QWidget* SceneWidget::indicator()
{
    return sharedIndicatorLabel;
}


void SceneWidget::Impl::initializeGL()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::initializeGL()" << endl;
    }

    renderer->setDefaultFramebufferObject(defaultFramebufferObject());
    
    if(renderer->initializeGL()){
        if(glslRenderer){
            auto& vendor = glslRenderer->glVendor();
            if(vendor.find("NVIDIA Corporation") != string::npos){
                needToClearGLOnFrameBufferChange = true;
            }
        }
    } else {
        MessageView::instance()->putln(
            _("OpenGL initialization failed."), MessageView::Error);
        // This view shoulbe be disabled when the glew initialization is failed.
    }
}


void SceneWidget::Impl::resizeGL(int width, int height)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::resizeGL()" << endl;
    }
    needToUpdateViewportInformation = true;
}


void SceneWidget::Impl::onSceneGraphUpdated(const SgUpdate& sgUpdate)
{
    if(sgUpdate.hasAction(SgUpdate::Added | SgUpdate::Removed)){
        if(sgUpdate.action() & SgUpdate::Added){
            if(auto node = sgUpdate.path().front()->toNode()){
                checkAddedEditableNodes(node);
            }
        } else if(sgUpdate.action() & SgUpdate::Removed){
            if(auto node = sgUpdate.path().front()->toNode()){
                checkRemovedEditableNodes(node);
            }
        }
        needToUpdatePreprocessedNodeTree = true;        
    }

    if(!isRendering){
        QOpenGLWidget::update();
    }
}


void SceneWidget::Impl::extractEditablesInSubTree(SgNode* node, vector<EditableNodeInfo>& editables)
{
    if(node->hasAttribute(SgObject::Operable)){
        if(auto editable = dynamic_cast<SceneWidgetEventHandler*>(node)){
            editables.emplace_back(node, editable);
        }
    }
    if(auto group = node->toGroupNode()){
        for(auto& child : *group){
            extractEditablesInSubTree(child, editables);
        }
    }
}


void SceneWidget::Impl::checkAddedEditableNodes(SgNode* node)
{
    if(pCurrentTmpEditableArray != tmpEditableArrays.begin()){
        warnRecursiveEditableNodeSetChange();
    }
    
    auto& addedEditables = *pCurrentTmpEditableArray;
    extractEditablesInSubTree(node, addedEditables);

    if(!addedEditables.empty()){

        ++pCurrentTmpEditableArray;
        if(pCurrentTmpEditableArray == tmpEditableArrays.end()){
            tmpEditableArrays.emplace_back();
            pCurrentTmpEditableArray = tmpEditableArrays.end();
            --pCurrentTmpEditableArray;
        }

        for(auto& editable : addedEditables){
            editable.handler->onSceneModeChanged(&latestEvent);
        }
        addedEditables.clear();

        --pCurrentTmpEditableArray;
    }
}


void SceneWidget::Impl::checkRemovedEditableNodes(SgNode* node)
{
    if(pCurrentTmpEditableArray != tmpEditableArrays.begin()){
        warnRecursiveEditableNodeSetChange();
    }
    
    auto& removedEditables = *pCurrentTmpEditableArray;
    extractEditablesInSubTree(node, removedEditables);

    if(!removedEditables.empty()){

        ++pCurrentTmpEditableArray;
        if(pCurrentTmpEditableArray == tmpEditableArrays.end()){
            tmpEditableArrays.emplace_back();
            pCurrentTmpEditableArray = tmpEditableArrays.end();
            --pCurrentTmpEditableArray;
        }

        list<EditableNodeInfo> editablesToClearFocus;
        for(auto& editable : removedEditables){
            if(editable == lastMouseMovedEditable){
                lastMouseMovedEditable.handler->onPointerLeaveEvent(&latestEvent);
                lastMouseMovedEditable.clear();
            }
            if(!focusedEditablePath.empty()){
                auto iter = focusedEditablePath.begin();
                while(iter != focusedEditablePath.end()){
                    auto& focused = *iter;
                    if(focused == editable){
                        iter = focusedEditablePath.erase(iter);
                        editablesToClearFocus.push_back(focused);
                    } else {
                        ++iter;
                    }
                }
            }
            if(focusedEditablePath.empty() && !lastMouseMovedEditable){
                break;
            }
        }

        for(auto& editable : editablesToClearFocus){
            editable.handler->onFocusChanged(&latestEvent, false);
        }

        removedEditables.clear();

        --pCurrentTmpEditableArray;
    }
}
    

void SceneWidget::Impl::warnRecursiveEditableNodeSetChange()
{
    string message;
    auto name = self->objectName().toStdString();
    if(name.empty()){
        message = _("Recursive editable node set change on a scene widget.");
    } else {
        message = fmt::format(_("Recursive editable node set change on scene widget {0}."), name);
    }
    MessageView::instance()->putln(message, MessageView::Warning);
}


void SceneWidget::Impl::paintGL()
{
    if(TRACE_FUNCTIONS){
        static int counter = 0;
        cout << "SceneWidget::Impl::paintGL() " << counter++ << endl;
    }

    auto newFramebuffer = defaultFramebufferObject();
    if(newFramebuffer != prevDefaultFramebufferObject){
        /**
           For NVIDIA GPUs, GLSLSceneRenderer may not be able to render properly
           when the placement or some other configurations of QOpenGLWidget used
           with the renderer change. To avoid the problem, the OpenGL resources
           used in the renderer should be cleared when the changes occur, and the
           resources should be recreated in the new configurations. This is done
           by the following code. The configuration changes can be detected by
           checking the ID of the default frame buffer object.
           
           \todo The view layout change in loading a project should be done before
           loading any items to avoid unnecessary re-initializations of the OpenGL
           resources to reduce the overhead.
        */
        if(needToClearGLOnFrameBufferChange && prevDefaultFramebufferObject > 0){
            renderer->clearGL();
            MessageView::instance()->putln(
                fmt::format(_("The OpenGL resources of {0} has been cleared."),
                            self->objectName().toStdString()));
        }

        // The default FBO must be updated after the clearGL function
        renderer->setDefaultFramebufferObject(newFramebuffer);
        
        prevDefaultFramebufferObject = newFramebuffer;
    }

    if(needToUpdateViewportInformation){
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);
        renderer->updateViewportInformation(viewport[0], viewport[1], viewport[2], viewport[3]);
        needToUpdateViewportInformation = false;
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

    isRendering = true;
    renderer->render();
    isRendering = false;

    if(fpsTimer.isActive()){
        renderFPS();
    }
}


void SceneWidget::Impl::tryToResumeNormalRendering()
{
    if(isDraggingView()){
        timerToRenderNormallyAfterInteractiveCameraPositionChange.start(0);
    } else {
        update();
    }
}


void SceneWidget::Impl::renderFPS()
{
    /*
    renderer->setColor(Vector3f(1.0f, 1.0f, 1.0f));
    renderText(20, 20, QString("FPS: %1").arg(fps));
    */
    fpsRendered = true;
    ++fpsCounter;
}


void SceneWidget::Impl::onFPSUpdateRequest()
{
    double oldFps = fps;
    fps = fpsCounter / 0.5;
    fpsCounter = 0;
    fpsRendered = false;
    if(oldFps > 0.0 || fps > 0.0){
        fpsRenderingTimer.start(100);
    }
}
    

void SceneWidget::Impl::onFPSRenderingRequest()
{
    if(!fpsRendered){
        update();
        --fpsCounter;
    }
}


void SceneWidget::Impl::showFPS(bool on)
{
    if(on){
        fpsCounter = 0;
        fpsTimer.start(500);
    } else {
        fpsTimer.stop();
    }
    onFPSUpdateRequest();
}


void SceneWidget::Impl::onFPSTestButtonClicked()
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
        

void SceneWidget::Impl::doFPSTest()
{
    if(!isBuiltinCameraCurrent){
        showWarningDialog(
            _("FPS test cannot be executed because the current camera is not a built-in interactive camera."));
        return;
    }
    
    isDoingFPSTest = true;
    isFPSTestCanceled = false;
    
    const Vector3 p = lastClickedPoint;
    const Isometry3 C = builtinCameraTransform->T();

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

    update();

    QMessageBox::information(config, _("FPS Test Result"),
                             QString(_("FPS: %1 frames / %2 [s] = %3")).arg(numFrames).arg(time).arg(fps));

    builtinCameraTransform->setTransform(C);
    update();

    isDoingFPSTest = false;
}


void SceneWidget::setCursor(const QCursor cursor)
{
    impl->setCursor(cursor);
}


void SceneWidget::Impl::resetCursor()
{
    self->setCursor(isEditMode ? editModeCursor : defaultCursor);
}


SignalProxy<void()> SceneWidget::sigStateChanged() const
{
    return impl->sigStateChanged;
}


void SceneWidget::setEditMode(bool on)
{
    impl->setEditMode(on, true);
}


void SceneWidget::Impl::setEditMode(bool on, bool doAdvertise)
{
    if(on != isEditMode){
        isEditMode = on;
        resetCursor();
        sharedIndicatorLabel->clear();
        if(doAdvertise){
            advertiseSceneModeChange(true);
        }
    }
}


void SceneWidget::Impl::toggleEditMode()
{
    setEditMode(!isEditMode, true);
}


bool SceneWidget::isEditMode() const
{
    return impl->isEditMode;
}


void SceneWidget::Impl::advertiseSceneModeChange(bool doModeSyncRequest)
{
    if(!isEditMode){
        // Clear focus
        for(auto& editable : focusedEditablePath){
            editable.handler->onFocusChanged(&latestEvent, false);
        }
    }
    
    if(activeCustomModeHandler){
        activeCustomModeHandler->onSceneModeChanged(&latestEvent);
    }
    
    advertiseSceneModeChangeInSubTree(sceneRoot);
    
    if(isEditMode){
        for(auto& element : focusedEditablePath){
            element.handler->onFocusChanged(&latestEvent, true);
        }
        if(lastMouseMoveEvent){
            mouseMoveEvent(lastMouseMoveEvent);
        }
    }
    
    if(doModeSyncRequest && isModeSyncEnabled){
        isEditModeInModeSync = isEditMode;
        isHighlightingEnabledInModeSync = isHighlightingEnabled;
        ::sigModeSyncRequest(self);
    }

    emitSigStateChangedLater();
}


void SceneWidget::Impl::advertiseSceneModeChangeInSubTree(SgNode* node)
{
    if(node->hasAttribute(SgObject::Operable)){
        if(auto editable = dynamic_cast<SceneWidgetEventHandler*>(node)){
            editable->onSceneModeChanged(&latestEvent);
        }
    }
    if(auto group = node->toGroupNode()){
        for(auto& child : *group){
            advertiseSceneModeChangeInSubTree(child);
        }
    }
}


int SceneWidget::issueUniqueCustomModeId()
{
    static int id = 2;
    return id++;
}


void SceneWidget::activateCustomMode(SceneWidgetEventHandler* modeHandler, int modeId)
{
    auto prevHandler = impl->activeCustomModeHandler;
    impl->activeCustomModeHandler = modeHandler;
    impl->activeCustomModeId = modeHandler ? modeId : 0;

    if(modeHandler != prevHandler){
        impl->resetCursor();
        if(prevHandler){
            prevHandler->onSceneModeChanged(&impl->latestEvent);
        }
        if(modeHandler){
            modeHandler->onSceneModeChanged(&impl->latestEvent);
        }
        impl->sigStateChanged();
    }
}


SceneWidgetEventHandler* SceneWidget::activeCustomModeHandler()
{
    return impl->activeCustomModeHandler;
}


int SceneWidget::activeCustomMode() const
{
    return impl->activeCustomModeId;
}


void SceneWidget::deactivateCustomMode(SceneWidgetEventHandler* modeHandler)
{
    if(!modeHandler || modeHandler == impl->activeCustomModeHandler){
        activateCustomMode(nullptr, 0);
    }
}


SceneWidgetEvent* SceneWidget::latestEvent()
{
    return &impl->latestEvent;
}


Vector3 SceneWidget::lastClickedPoint() const
{
    return impl->lastClickedPoint;
}


void SceneWidget::setViewpointOperationMode(ViewpointOperationMode mode)
{
    impl->viewpointOperationMode.select(mode);
    impl->emitSigStateChangedLater();
}


SceneWidget::ViewpointOperationMode SceneWidget::viewpointOperationMode() const
{
    return static_cast<ViewpointOperationMode>(impl->viewpointOperationMode.which());
}


void SceneWidget::viewAll()
{
    impl->viewAll();
}


void SceneWidget::Impl::viewAll()
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
    
    Isometry3& T = interactiveCameraTransform->T();
    T.translation() +=
        (bbox.center() - T.translation())
        + T.rotation() * Vector3(0, 0, 2.0 * radius * builtinPersCamera->nearClipDistance() / length);


    if(auto ortho = dynamic_cast<SgOrthographicCamera*>(renderer->currentCamera())){
        if(a >= 1.0){
            ortho->setHeight(radius * 2.0);
        } else {
            ortho->setHeight(radius * 2.0 / a);
        }
        ortho->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));

    } else {
        interactiveCameraTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
    }
}


void SceneWidget::Impl::showPickingImageWindow()
{
    auto glslRenderer = dynamic_cast<GLSLSceneRenderer*>(renderer);
    if(glslRenderer){
        if(!pickingImageWindow){
            auto vp = renderer->viewport();
            pickingImageWindow = new ImageWindow(vp[2], vp[3]);
        }
        pickingImageWindow->show();
    }
}


void SceneWidget::Impl::onUpsideDownToggled(bool on)
{
    renderer->setUpsideDown(on);
    update();
}


void SceneWidget::Impl::updateLatestEvent(QKeyEvent* event)
{
    latestEvent.modifiers_ = event->modifiers();
    latestEvent.key_ = event->key();
}


void SceneWidget::Impl::updateLatestEvent(int x, int y, int modifiers)
{
    latestEvent.x_ = x;
    latestEvent.y_ = height() - y - 1;
    latestEvent.modifiers_ = modifiers;
}


void SceneWidget::Impl::updateLatestEvent(QMouseEvent* event)
{
    updateLatestEvent(event->x(), event->y(), event->modifiers());
    latestEvent.button_ = event->button();
    latestGlobalMousePos = event->globalPos();
}


void SceneWidget::Impl::updateLatestEventPath(bool forceFullPicking)
{
    if(needToUpdateViewportInformation ||
       (!forceFullPicking && isLightweightViewChangeEnabled)){
        return;
    }
    
    bool isPickingVisualizationEnabled = pickingImageWindow && pickingImageWindow->isVisible();
    renderer->setPickingImageOutputEnabled(isPickingVisualizationEnabled);

    makeCurrent();

    const int r = devicePixelRatio();
    int px = r * latestEvent.x();
    int py = r * latestEvent.y();

    isRendering = true;
    bool picked = renderer->pick(px, py);
    isRendering = false;

    if(isPickingVisualizationEnabled){
        Image image;
        if(renderer->getPickingImage(image)){
            pickingImageWindow->setImage(image, px, image.height() - py - 1);
        }
    }

    doneCurrent();

    latestEvent.nodePath_.clear();
    latestEvent.pixelSizeRatio_ = 0.0;
    pointedEditablePath.clear();

    if(picked){
        latestEvent.point_ = renderer->pickedPoint();
        latestEvent.pixelSizeRatio_ = renderer->projectedPixelSizeRatio(latestEvent.point_);
        latestEvent.nodePath_ = renderer->pickedNodePath();

        for(auto& node : latestEvent.nodePath_){
            if(auto editable = dynamic_cast<SceneWidgetEventHandler*>(node)){
                pointedEditablePath.emplace_back(node, editable);
            }
        }
    }
}


void SceneWidget::Impl::updateLastClickedPoint()
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
EditableNodeInfo SceneWidget::Impl::applyEditableFunction
(vector<EditableNodeInfo>& editablePath, std::function<bool(SceneWidgetEventHandler* editable)> function)
{
    for(auto p = editablePath.rbegin(); p != editablePath.rend(); ++p){
        auto& editableNode = *p;
        if(function(editableNode.handler)){
            return editableNode;
        }
    }
    return EditableNodeInfo(); // null
}
    

bool SceneWidget::Impl::setFocusToEditablePath(vector<EditableNodeInfo>& editablePath)
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

    focusedEditable.clear();
    vector<EditableNodeInfo> tmpPath(focusedEditablePath);

    for(size_t i=indexOfFirstEditableToChangeFocus; i < tmpPath.size(); ++i){
        tmpPath[i].handler->onFocusChanged(&latestEvent, false);
    }
    for(size_t i=indexOfFirstEditableToChangeFocus; i < editablePath.size(); ++i){
        editablePath[i].handler->onFocusChanged(&latestEvent, true);
    }
    focusedEditablePath = editablePath;
    focusedEditable = editablePath.back();

    return true;
}


bool SceneWidget::Impl::setFocusToPointedEditablePath(const EditableNodeInfo& targetEditable)
{
    if(targetEditable){
        vector<EditableNodeInfo> path;
        for(auto& element : pointedEditablePath){
            path.push_back(element);
            if(element == targetEditable){
                return setFocusToEditablePath(path);
            }
        }
    }
    return false;
}


bool SceneWidget::setSceneFocus(const SgNodePath& path)
{
    vector<EditableNodeInfo> editablePath;
    for(auto& node : path){
        if(auto editable = dynamic_cast<SceneWidgetEventHandler*>(node)){
            editablePath.emplace_back(node, editable);
        }
    }
    return impl->setFocusToEditablePath(editablePath);
}


void SceneWidget::Impl::keyPressEvent(QKeyEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::keyPressEvent()" << endl;
    }

    /*
    if(isEditMode && event->key() == Qt::Key_Alt){
        setCursor(defaultCursor);
        return;
    }
    */
    
    updateLatestEvent(event);
    updateLatestEventPath();

    bool handled = false;

    if(isEditMode){
        if(applyEditableFunction(
               focusedEditablePath,
               [&](SceneWidgetEventHandler* editable){ return editable->onKeyPressEvent(&latestEvent); })){
            handled = true;
        }
    }

    if(!handled){
        switch(event->key()){

        case Qt::Key_Escape:
            toggleEditMode();
            handled = true;
            break;

        case Qt::Key_1:
            self->setViewpointOperationMode(FirstPersonMode);
            handled = true;
            break;
            
        case Qt::Key_3:
            self->setViewpointOperationMode(ThirdPersonMode);
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
        default:
            break;
        }
    }

    if(!handled){
        event->setAccepted(false);
    }
}


void SceneWidget::Impl::keyReleaseEvent(QKeyEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::keyReleaseEvent()" << endl;
    }

    /*
    if(isEditMode && event->key() == Qt::Key_Alt){
        setCursor(editModeCursor);
        return;
    }
    */

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
            handled = focusedEditable.handler->onKeyReleaseEvent(&latestEvent);
        }
    }

    if(!handled){
        event->setAccepted(false);
    }
}


void SceneWidget::Impl::mousePressEvent(QMouseEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::mousePressEvent()" << endl;
    }
    
    updateLatestEvent(event);
    updateLatestEventPath();
    updateLastClickedPoint();
    mousePressX = event->x();
    mousePressY = event->y();

    bool handled = false;
    bool forceViewMode = (event->modifiers() & Qt::AltModifier);

    if(isEditMode && !forceViewMode){
        if(activeCustomModeHandler){
            handled = activeCustomModeHandler->onButtonPressEvent(&latestEvent);
        }
        if(!handled){
            handled = setFocusToPointedEditablePath(
                applyEditableFunction(
                    pointedEditablePath,
                    [&](SceneWidgetEventHandler* editable){ return editable->onButtonPressEvent(&latestEvent); }));
            if(handled){
                dragMode = ABOUT_TO_EDIT;
            }
        }
    }

    if(!handled){
        if(event->button() == Qt::RightButton){
            if(isEditMode){
                showEditModePopupMenu(event->globalPos());
            } else {
                showViewModePopupMenu(event->globalPos());
            }
            handled = true;
        }
    }

    if(!handled){
        startViewChange();
    }
}


void SceneWidget::Impl::mouseDoubleClickEvent(QMouseEvent* event)
{
    updateLatestEvent(event);
    updateLatestEventPath();
    
    bool handled = false;
    if(isEditMode){
        if(activeCustomModeHandler){
            handled = activeCustomModeHandler->onDoubleClickEvent(&latestEvent);
        }
        if(!handled){
            handled = setFocusToPointedEditablePath(
                applyEditableFunction(
                    pointedEditablePath,
                    [&](SceneWidgetEventHandler* editable){ return editable->onDoubleClickEvent(&latestEvent); }));
        }
    }
    if(!handled){
        toggleEditMode();

        if(isEditMode){
            setFocusToPointedEditablePath(
                applyEditableFunction(
                    pointedEditablePath,
                    [&](SceneWidgetEventHandler* editable){
                        if(editable->onButtonPressEvent(&latestEvent)){
                            return editable->onButtonReleaseEvent(&latestEvent);
                        }
                        return false;
                    }));
        }
    }
}


void SceneWidget::Impl::mouseReleaseEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    bool handled = false;

    if(isEditMode){
        if(activeCustomModeHandler){
            handled = activeCustomModeHandler->onButtonReleaseEvent(&latestEvent);
        }
    }
    if(!handled){
        if(isEditMode && (dragMode == ABOUT_TO_EDIT || dragMode == EDITING)){
            if(focusedEditable){
                updateLatestEventPath();
                focusedEditable.handler->onButtonReleaseEvent(&latestEvent);
            }
        }
        dragMode = NO_DRAGGING;
    }
}


void SceneWidget::Impl::mouseMoveEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    bool handled = false;

    switch(dragMode){

    case ABOUT_TO_EDIT:
    case EDITING:
        if(focusedEditable){
            if(dragMode == ABOUT_TO_EDIT){
                const int thresh = ThreashToStartPointerMoveEventForEditableNode;
                if(abs(event->x() - mousePressX) > thresh || abs(event->y() - mousePressY) > thresh){
                    dragMode = EDITING;
                }
            }
            if(dragMode == EDITING){
                focusedEditable.handler->onPointerMoveEvent(&latestEvent);
            }
            handled = true;
        }
        break;
        
    case VIEW_ROTATION:
        dragViewRotation();
        handled = true;
        break;
        
    case VIEW_TRANSLATION:
        dragViewTranslation();
        handled = true;
        break;
        
    case VIEW_ZOOM:
        dragViewZoom();
        handled = true;
        break;
        
    default:
        break;
    }

    if(!handled){
        updateLatestEventPath();
        
        if(activeCustomModeHandler){
            handled = activeCustomModeHandler->onPointerMoveEvent(&latestEvent);
        }
        if(!handled && isEditMode){
            auto mouseMovedEditable =
                applyEditableFunction(
                    pointedEditablePath,
                    [&](SceneWidgetEventHandler* editable){ return editable->onPointerMoveEvent(&latestEvent); });

            if(mouseMovedEditable){
                handled = true;

                /**
                   The following code changes the input focus to this scene widget if the mouse pointer
                   is pointing an editable scene object and the event is processed by it.
                   This is sometimes convenient for operating the object with key inputs because it can
                   omit the operation to click the view or the object first. However, changing the focus
                   without clicking a view does not conform to the usual GUI operation style and can be
                   confusing. Thus the focus change should not be enabled.
                */
                /*
                if(!QWidget::hasFocus()){
                    QWidget::setFocus(Qt::MouseFocusReason);
                }
                */
            }
            if(lastMouseMovedEditable != mouseMovedEditable){
                if(!mouseMovedEditable){
                    resetCursor();
                }
                if(lastMouseMovedEditable){
                    lastMouseMovedEditable.handler->onPointerLeaveEvent(&latestEvent);
                }
                lastMouseMovedEditable = mouseMovedEditable;
            }
        }
    }

    if(!handled){
        if(latestEvent.nodePath().empty()){
            updateIndicator("");
        } else {
            static string f1(_("Global Position: ({0:.3f} {1:.3f} {2:.3f})"));
            static string f2(_("Object: {0}, Global Position: ({1:.3f} {2:.3f} {3:.3f})"));
            const Vector3& p = latestEvent.point();
            string name = findObjectNameFromNodePath(latestEvent.nodePath());
            if(name.empty()){
                updateIndicator(fmt::format(f1, p.x(), p.y(), p.z()));
            } else {
                updateIndicator(fmt::format(f2, name, p.x(), p.y(), p.z()));
            }
        }
    }

    if(lastMouseMoveEvent){
        delete lastMouseMoveEvent;
    }
    lastMouseMoveEvent =
        new QMouseEvent(
            event->type(), event->localPos(), event->windowPos(), event->screenPos(),
            event->button(), event->buttons(), event->modifiers());
}


string SceneWidget::Impl::findObjectNameFromNodePath(const SgNodePath& path)
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


void SceneWidget::Impl::findObjectNameFromChildren(SgObject* object, string& name)
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


void SceneWidget::Impl::leaveEvent(QEvent* event)
{
    if(lastMouseMovedEditable){
        lastMouseMovedEditable.handler->onPointerLeaveEvent(&latestEvent);
        lastMouseMovedEditable.clear();
    }
    if(lastMouseMoveEvent){
        delete lastMouseMoveEvent;
        lastMouseMoveEvent = nullptr;
    }
}


void SceneWidget::Impl::wheelEvent(QWheelEvent* event)
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::wheelEvent()" << endl;
        cout << "angleDelta().y(): " << event->angleDelta().y() << endl;
    }

    updateLatestEvent(event->x(), event->y(), event->modifiers());
    updateLatestEventPath();
    updateLastClickedPoint();

    const double s = event->angleDelta().y() / 8.0 / 15.0;
    latestEvent.wheelSteps_ = s;

    bool handled = false;
    if(isEditMode && !(event->modifiers() & Qt::AltModifier)){
        if(activeCustomModeHandler){
            handled = activeCustomModeHandler->onScrollEvent(&latestEvent);
        }
        if(!handled){
            handled = setFocusToPointedEditablePath(
                applyEditableFunction(
                    pointedEditablePath,
                    [&](SceneWidgetEventHandler* editable){ return editable->onScrollEvent(&latestEvent); }));
        }
    }    

    if(interactiveCameraTransform && !handled){
        if(event->orientation() == Qt::Vertical){
            zoomView(0.25 * s);
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


void SceneWidget::Impl::focusInEvent(QFocusEvent*)
{
    sigWidgetFocusChanged(true);
}


void SceneWidget::Impl::focusOutEvent(QFocusEvent*)
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
Isometry3 SceneWidget::Impl::getNormalizedCameraTransform(const Isometry3& T)
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
        
    Isometry3 N;
    N.linear() << x, y, z;
    N.translation() = T.translation();
    return N;
}


void SceneWidget::Impl::startViewChange()
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
       

void SceneWidget::Impl::startViewRotation()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::startViewRotation()" << endl;
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


void SceneWidget::Impl::dragViewRotation()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::dragViewRotation()" << endl;
    }

    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }
    
    const double dx = latestEvent.x() - orgMouseX;
    const double dy = latestEvent.y() - orgMouseY;

    Isometry3 R;
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
        Isometry3& T = interactiveCameraTransform->T();
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


void SceneWidget::Impl::startViewTranslation()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::startViewTranslation()" << endl;
    }

    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }

    const Isometry3& C = interactiveCameraTransform->T();

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


void SceneWidget::Impl::dragViewTranslation()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::dragViewTranslation()" << endl;
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


void SceneWidget::Impl::startViewZoom()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::startViewZoom()" << endl;
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


void SceneWidget::Impl::dragViewZoom()
{
    if(TRACE_FUNCTIONS){
        cout << "SceneWidget::Impl::dragViewZoom()" << endl;
    }

    SgCamera* camera = renderer->currentCamera();
    
    const double dy = latestEvent.y() - orgMouseY;
    const double ratio = expf(dy * 0.01);

    if(dynamic_cast<SgPerspectiveCamera*>(camera)){
        const Isometry3& C = orgCameraPosition;
        const Vector3 v = SgCamera::direction(C);

        if(isFirstPersonMode()){
            double speed = 0.02;
            interactiveCameraTransform->setTranslation(C.translation() + speed * dy * v);
            
        } else {
            const double l0 = (lastClickedPoint - C.translation()).dot(v);
            interactiveCameraTransform->setTranslation(C.translation() + v * (l0 * (-ratio + 1.0)));
        }

        notifyCameraPositionInteractivelyChanged();

    } else if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
        ortho->setHeight(orgOrthoCameraHeight * ratio);
        ortho->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
    }
}


void SceneWidget::Impl::zoomView(double ratio)
{
    if(!interactiveCameraTransform){
        dragMode = NO_DRAGGING;
        return;
    }

    SgCamera* camera = renderer->currentCamera();
    if(dynamic_cast<SgPerspectiveCamera*>(camera)){
        const Isometry3& C = interactiveCameraTransform->T();
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

        notifyCameraPositionInteractivelyChanged();

    } else if(auto ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
        ortho->setHeight(ortho->height() * expf(ratio));
        ortho->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
    }
}


void SceneWidget::Impl::notifyCameraPositionInteractivelyChanged()
{
    isCameraPositionInteractivelyChanged = true;
    interactiveCameraTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
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


void SceneWidget::Impl::rotateBuiltinCameraView(double dPitch, double dYaw)
{
    const Isometry3 T = builtinCameraTransform->T();
    builtinCameraTransform->setTransform(
        getNormalizedCameraTransform(
            Translation3(cameraViewChangeCenter) *
            AngleAxis(dYaw, Vector3::UnitZ()) *
            AngleAxis(dPitch, SgCamera::right(T)) *
            Translation3(-cameraViewChangeCenter) *
            T));

    builtinCameraTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
}


void SceneWidget::translateBuiltinCameraView(const Vector3& dp_local)
{
    impl->translateBuiltinCameraView(dp_local);
}


void SceneWidget::Impl::translateBuiltinCameraView(const Vector3& dp_local)
{
    const Isometry3 T = builtinCameraTransform->T();
    builtinCameraTransform->setTransform(
        getNormalizedCameraTransform(
            Translation3(T.linear() * dp_local) * T));
    builtinCameraTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
}


void SceneWidget::Impl::showViewModePopupMenu(const QPoint& globalPos)
{
    menuManager.setNewPopupMenu(self);
    
    sigContextMenuRequest(&latestEvent, &menuManager);

    menuManager.setPath("/");
    menuManager.addItem(_("Edit Mode")) ->sigTriggered().connect([&](){ toggleEditMode(); });

    menuManager.popupMenu()->popup(globalPos);
}


void SceneWidget::Impl::showEditModePopupMenu(const QPoint& globalPos)
{
    menuManager.setNewPopupMenu(self);

    int prevNumItems = 0;

    bool handled = false;
    
    if(activeCustomModeHandler){
        handled = activeCustomModeHandler->onContextMenuRequest(&latestEvent, &menuManager);
    }

    if(!handled && !pointedEditablePath.empty()){
        auto editableToFocus = pointedEditablePath.front();
        for(auto p = pointedEditablePath.rbegin(); p != pointedEditablePath.rend(); ++p){
            auto& editableNode = *p;
            handled = editableNode.handler->onContextMenuRequest(&latestEvent, &menuManager);
            if(handled){
                int numItems = menuManager.numItems();
                if(numItems > prevNumItems){
                    menuManager.addSeparator();
                    prevNumItems = numItems;
                    editableToFocus = editableNode;
                }
            }
        }
        setFocusToPointedEditablePath(editableToFocus);
    }

    sigContextMenuRequest(&latestEvent, &menuManager);
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


void SceneWidget::showContextMenuAtPointerPosition()
{
    if(impl->isEditMode){
        impl->showEditModePopupMenu(impl->latestGlobalMousePos);
    } else {
        impl->showViewModePopupMenu(impl->latestGlobalMousePos);
    }
}


SignalProxy<void(SceneWidgetEvent* event, MenuManager* menuManager)>
SceneWidget::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}


void SceneWidget::Impl::showBackgroundColorDialog()
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


void SceneWidget::Impl::showGridColorDialog(int index)
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


void SceneWidget::Impl::showDefaultColorDialog()
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


InteractiveCameraTransform* SceneWidget::findOwnerInteractiveCameraTransform(int cameraIndex)
{
    const SgNodePath& path = impl->renderer->cameraPath(cameraIndex);
    for(size_t i=0; i < path.size() - 1; ++i){
        if(InteractiveCameraTransform* transform = dynamic_cast<InteractiveCameraTransform*>(path[i])){
            return transform;
        }
    }
    return nullptr;
}


void SceneWidget::Impl::onCurrentCameraChanged()
{
    interactiveCameraTransform.reset();
    isBuiltinCameraCurrent = false;
    
    SgCamera* current = renderer->currentCamera();

    if(!current){
        latestEvent.cameraIndex_ = -1;
        latestEvent.cameraPath_.clear();
    } else {
        int index = renderer->currentCameraIndex();
        const SgNodePath& path = renderer->cameraPath(index);
        for(int i = path.size() - 2; i >= 0; --i){
            interactiveCameraTransform = dynamic_cast<InteractiveCameraTransform*>(path[i]);
            if(interactiveCameraTransform){
                isBuiltinCameraCurrent = (current == builtinPersCamera || current == builtinOrthoCamera);
                break;
            }
        }
        latestEvent.cameraIndex_ = index;
        latestEvent.cameraPath_ = path;
    }

    if(!isRendering){
        update();
    }
}


void SceneWidget::Impl::onTextureToggled(bool on)
{
    renderer->enableTexture(on);
    update();
}


void SceneWidget::Impl::onLineWidthChanged(double width)
{
    renderer->setDefaultLineWidth(width);
    update();
}


void SceneWidget::Impl::onPointSizeChanged(double size)
{
    renderer->setDefaultPointSize(size);
    update();
}


void SceneWidget::setVisiblePolygonElements(int elementFlags)
{
    impl->setVisiblePolygonElements(elementFlags);
}


void SceneWidget::Impl::setVisiblePolygonElements(int elementFlags)
{
    int currentFlags = visiblePolygonElements();
    if(elementFlags != currentFlags){
        if(!polygonDrawStyle){
            polygonDrawStyle = new SgPolygonDrawStyle;
        }
        bool notified = false;
        polygonDrawStyle->setPolygonElements(elementFlags);

        if(!polygonDrawStyle->hasParents() && elementFlags != SgPolygonDrawStyle::Face){
            sceneRoot->removeChild(scene);
            polygonDrawStyle->addChild(scene);
            SgUpdate tmpUpdate;
            sceneRoot->insertChild(0, polygonDrawStyle, tmpUpdate);
            notified = true;
        }
        
        if(!notified){
            polygonDrawStyle->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
        }
        emitSigStateChangedLater();
    }
}
    

int SceneWidget::visiblePolygonElements() const
{
    return impl->visiblePolygonElements();
}


int SceneWidget::Impl::visiblePolygonElements() const
{
    return polygonDrawStyle ? polygonDrawStyle->polygonElements() : SgPolygonDrawStyle::Face;
}


void SceneWidget::setHighlightingEnabled(bool on)
{
    if(on != impl->isHighlightingEnabled){
        impl->isHighlightingEnabled = on;
        impl->advertiseSceneModeChange(true);
    }
}


bool SceneWidget::isHighlightingEnabled() const
{
    return impl->isHighlightingEnabled;
}


void SceneWidget::setCollisionLineVisibility(bool on)
{
    impl->setCollisionLineVisibility(on);
}


void SceneWidget::Impl::setCollisionLineVisibility(bool on)
{
    if(on != collisionLineVisibility){
        collisionLineVisibility = on;
        renderer->setProperty(SceneRenderer::PropertyKey("collisionLineRatio"), on ? 50.0 : 0.0);
        update();
        emitSigStateChangedLater();
    }
}


bool SceneWidget::collisionLineVisibility() const
{
    return impl->collisionLineVisibility;
}


void SceneWidget::Impl::onFieldOfViewChanged(double fov)
{
    config->builtinCameraConnections.block();
    builtinPersCamera->setFieldOfView(fov);
    builtinPersCamera->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
    config->builtinCameraConnections.unblock();
}


void SceneWidget::Impl::onClippingDepthChanged()
{
    config->builtinCameraConnections.block();
    double zNear = config->zNearSpin.value();
    double zFar = config->zFarSpin.value();
    builtinPersCamera->setNearClipDistance(zNear);
    builtinPersCamera->setFarClipDistance(zFar);
    builtinOrthoCamera->setNearClipDistance(zNear);
    builtinOrthoCamera->setFarClipDistance(zFar);
    sgUpdate.setAction(SgUpdate::Modified);
    builtinPersCamera->notifyUpdate(sgUpdate);
    builtinOrthoCamera->notifyUpdate(sgUpdate);
    config->builtinCameraConnections.unblock();
}


void SceneWidget::Impl::onSmoothShadingToggled(bool on)
{
    renderer->setDefaultSmoothShading(on);
    update();
}


void SceneWidget::Impl::updateDefaultLights()
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

    worldLight->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
}


void SceneWidget::Impl::onNormalVisualizationChanged()
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
    return impl->grabFramebuffer().save(filename.c_str());
}


QImage SceneWidget::getImage()
{
    return impl->grabFramebuffer();
}


void SceneWidget::setScreenSize(int width, int height)
{
    impl->setScreenSize(width, height);
}


void SceneWidget::Impl::setScreenSize(int width, int height)
{
    QRect r = self->geometry();
    setGeometry((r.width() - width) / 2, (r.height() - height) / 2, width, height);
}


void SceneWidget::updateIndicator(const std::string& text)
{
    sharedIndicatorLabel->setText(text.c_str());
}


void SceneWidget::Impl::updateIndicator(const std::string& text)
{
    sharedIndicatorLabel->setText(text.c_str());
}


bool SceneWidget::storeState(Archive& archive)
{
    return impl->storeState(archive);
}


bool SceneWidget::Impl::storeState(Archive& archive)
{
    archive.write("editMode", isEditMode);
    archive.write("viewpointOperationMode", viewpointOperationMode.selectedSymbol());

    auto vpeList = archive.createFlowStyleListing("visible_polygon_elements");
    int vpe = self->visiblePolygonElements();
    if(vpe & PolygonFace){
        vpeList->append("face");
    }
    if(vpe & PolygonEdge){
        vpeList->append("edge");
    }
    if(vpe & PolygonVertex){
        vpeList->append("vertex");
    }

    archive.write("highlighting", isHighlightingEnabled);
    archive.write("collisionLines", collisionLineVisibility);

    config->storeState(archive);

    ListingPtr cameraListing = new Listing;
    set<SgPosTransform*> storedTransforms;
    int numCameras = renderer->numCameras();
    for(int i=0; i < numCameras; ++i){
        Mapping* cameraState = nullptr;
        if(InteractiveCameraTransform* transform = self->findOwnerInteractiveCameraTransform(i)){
            if(!storedTransforms.insert(transform).second){
                transform = nullptr; // already stored
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


void SceneWidget::Impl::writeCameraPath(Mapping& archive, const std::string& key, int cameraIndex)
{
    vector<string> cameraStrings = renderer->simplifiedCameraPathStrings(cameraIndex);
    if(cameraStrings.size() == 1){
        archive.write(key, cameraStrings.front());
    } else if(cameraStrings.size() >= 2){
        Listing& pathNode = *archive.createListing(key);
        pathNode.setFlowStyle(true);
        for(size_t i=0; i < cameraStrings.size(); ++i){
            pathNode.append(cameraStrings[i]);
        }
    }
}


Mapping* SceneWidget::Impl::storeCameraState(int cameraIndex, bool isInteractiveCamera, SgPosTransform* cameraTransform)
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
            const Isometry3& T = cameraTransform->T();
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


bool SceneWidget::Impl::restoreState(const Archive& archive)
{
    bool doUpdate = false;

    setEditMode(archive.get("editMode", isEditMode), false);
    
    string symbol;
    if(archive.read("viewpointOperationMode", symbol)){
        self->setViewpointOperationMode(
            ViewpointOperationMode(viewpointOperationMode.index(symbol)));
    }

    auto& vpeList = *archive.findListing("visible_polygon_elements");
    if(vpeList.isValid()){
        int vpe = 0;
        for(auto& node : vpeList){
            const auto& symbol = node->toString();
            if(symbol == "face"){
                vpe |= PolygonFace;
            } else if(symbol == "edge"){
                vpe |= PolygonEdge;
            } else if(symbol == "vertex"){
                vpe |= PolygonVertex;
            }
        }
        if(vpe){
            setVisiblePolygonElements(vpe);
        }
    }

    archive.read("highlighting", isHighlightingEnabled);
    setCollisionLineVisibility(archive.get("collisionLines", collisionLineVisibility));
    
    config->restoreState(archive);

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

    const Listing& cameraListing = *archive.findListing("cameras");
    if(cameraListing.isValid()){
        if(!restoreCameraStates(cameraListing, false)){
            archive.addPostProcess([&](){ restoreCameraStates(cameraListing, true); }, 1);
        }
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

            builtinCameraTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
            doUpdate = true;
            
            archive.addPostProcess([&](){ restoreCurrentCamera(cameraData); }, 1);
        }
    }

    if(doUpdate){
        updateGrids();
        update();
    }

    advertiseSceneModeChange(true);

    return true;
}


/**
   \todo In the second trial, the camera states that have aready restored in the first trial
   should not be restored twice.
*/
bool SceneWidget::Impl::restoreCameraStates(const Listing& cameraListing, bool isSecondTrial)
{
    bool restored = false;

    renderer->extractPreprocessedNodes();
    
    for(int i=0; i < cameraListing.size(); ++i){

        bool updated = false;
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
                        transform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
                    }
                }
                updated = true;
            }

            SgCamera* camera = renderer->camera(cameraIndex);
            if(SgPerspectiveCamera* pers = dynamic_cast<SgPerspectiveCamera*>(camera)){
                double fov;
                if(state.read("fieldOfView", fov)){
                    pers->setFieldOfView(fov);
                    updated = true;
                }
            }
            if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
                double height;
                if(state.read("orthoHeight", height)){
                    ortho->setHeight(height);
                    updated = true;
                }
            }
            double near, far;
            if(state.read("near", near)){
                camera->setNearClipDistance(near);
                updated = true;
            }
            if(state.read("far", far)){
                camera->setFarClipDistance(far);
                updated = true;
            }
            if(updated){
                camera->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
            }

            if(state.get("isCurrent", false)){
                renderer->setCurrentCamera(cameraIndex);
                restored = true;
                if(isSecondTrial){
                    update();
                }
            }
        }
    }

    return restored;
}


int SceneWidget::Impl::readCameraPath(const Mapping& archive, const char* key)
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
void SceneWidget::Impl::restoreCurrentCamera(const Mapping& cameraData)
{
    renderer->extractPreprocessedNodes();
    int index = readCameraPath(cameraData, "current");
    if(index >= 0){
        renderer->setCurrentCamera(index);
    }
}


void SceneWidget::Impl::updateGrids()
{
    if(gridGroup){
        activateSystemNode(gridGroup, false);
        gridGroup = nullptr;
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


SgLineSet* SceneWidget::Impl::createGrid(int index)
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
    SgIndexArray& vertexIndices = grid->lineVertexIndices();
    vertexIndices.resize(n);
    for(int i=0; i < n; ++i){
        vertexIndices[i] = i;
    }

    return grid;
}


void SceneWidget::Impl::activateSystemNode(SgNode* node, bool on)
{
    if(on){
        systemGroup->addChild(node, sgUpdate);
    } else {
        systemGroup->removeChild(node, sgUpdate);
    }
}


namespace {

ConfigDialog::ConfigDialog(SceneWidget::Impl* impl)
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
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Camera"))));
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

    hbox->addSpacing(8);
    upsideDownCheck.setText(_("Upside down"));
    upsideDownCheck.setChecked(false);
    upsideDownCheck.sigToggled().connect([=](bool on){ impl->onUpsideDownToggled(on); });
    hbox->addWidget(&upsideDownCheck);
    
    hbox->addStretch();
    vbox->addLayout(hbox);

    updateDefaultLightsLater.setFunction([=](){ impl->updateDefaultLights(); });
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Lighting"))));
    hbox = new QHBoxLayout;
    hbox->addWidget(new QLabel(_("Lighting mode")));

    normalLightingRadio.setText(_("Normal"));
    normalLightingRadio.setChecked(true);
    lightingModeGroup.addButton(&normalLightingRadio, GLSceneRenderer::NormalLighting);
    lightingMode.setSymbol(GLSceneRenderer::NormalLighting, "normal");
    hbox->addWidget(&normalLightingRadio);

    minLightingRadio.setText(_("Minimum"));
    lightingModeGroup.addButton(&minLightingRadio, GLSceneRenderer::MinimumLighting);
    lightingMode.setSymbol(GLSceneRenderer::MinimumLighting, "minimum");
    hbox->addWidget(&minLightingRadio);
    
    solidColorLightingRadio.setText(_("Solid color"));
    lightingModeGroup.addButton(&solidColorLightingRadio, GLSceneRenderer::SolidColorLighting);
    lightingMode.setSymbol(GLSceneRenderer::SolidColorLighting, "solid_color");
    hbox->addWidget(&solidColorLightingRadio);

    lightingModeGroup.sigButtonToggled().connect(
        [&, renderer](int mode, bool checked){
            if(checked){
                lightingMode.select(mode);
                renderer->setLightingMode((GLSceneRenderer::LightingMode)mode);
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

    auto pickingImageButton = new PushButton(_("Show the image for picking (for debug)"));
    pickingImageButton->sigClicked().connect([=](){ impl->showPickingImageWindow(); });
    hbox->addWidget(pickingImageButton);
    
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

}
