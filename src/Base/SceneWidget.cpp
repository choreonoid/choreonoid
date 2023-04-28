#include "SceneWidget.h"
#include "SceneBar.h"
#include "SceneWidgetEventHandler.h"
#include "InteractiveCameraTransform.h"
#include "Archive.h"
#include "MessageView.h"
#include "ExtensionManager.h"
#include "MenuManager.h"
#include "LazyCaller.h"
#include "Timer.h"
#include "AppConfig.h"
#include "DisplayValueFormat.h"
#include <cnoid/GL1SceneRenderer>
#include <cnoid/GLSLSceneRenderer>
#include <cnoid/Selection>
#include <cnoid/EigenArchive>
#include <cnoid/SceneCameras>
#include <cnoid/SceneLights>
#include <cnoid/SceneEffects>
#include <cnoid/CoordinateAxesOverlay>
#include <cnoid/ConnectionSet>
#include <QOpenGLWidget>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QElapsedTimer>
#include <QMessageBox>
#include <QCoreApplication>
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

bool isVerticalSyncMode_ = false;
bool isLowMemoryConsumptionMode_ = false;
Signal<void(bool on)> sigLowMemoryConsumptionModeChanged;

QLabel* sharedIndicatorLabel = nullptr;

Signal<void(SceneWidget* instance)> sigSceneWidgetCreated_;
Signal<void(SceneWidget* requester)> sigModeSyncRequest_;
bool isEditModeInModeSync = false;
bool isHighlightingEnabledInModeSync = false;


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
    GL1SceneRenderer* gl1Renderer;
    float lastDevicePixelRatio;
    GLuint prevDefaultFramebufferObject;
    bool isRendering;
    bool needToUpdatePreprocessedNodeTree;
    bool needToClearGLOnFrameBufferChange;
    SgUpdate sgUpdate;

    InteractiveCameraTransformPtr interactiveCameraTransform;

    bool hasActiveInteractiveCamera() const {
        return interactiveCameraTransform &&
            !interactiveCameraTransform->isInteractiveViewpointChangeLocked();
    }
    
    InteractiveCameraTransformPtr builtinCameraTransform;
    SgPerspectiveCameraPtr builtinPersCamera;
    SgOrthographicCameraPtr builtinOrthoCamera;
    int numBuiltinCameras;
    bool isBuiltinCameraCurrent;
    bool isLightweightViewChangeEnabled;
    bool isCameraPositionInteractivelyChanged;
    bool isCameraRollRistricted;
    int verticalAxis;
    Timer timerToRenderNormallyAfterInteractiveCameraPositionChange;

    Signal<void()> sigStateChanged;
    LazyCaller emitSigStateChangedLater;

    bool needToUpdateViewportInformation;
    bool isEditMode;
    bool isHighlightingEnabled;
    bool isModeSyncEnabled;
    ScopedConnection modeSyncConnection;
    std::set<ReferencedPtr> editModeBlockRequesters;

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
    NodeEventHandler customNodeEventHandler;

    SceneWidgetEventHandler* activeCustomModeHandler;
    int activeCustomModeId;

    bool collisionLineVisibility;

    ref_ptr<CoordinateAxesOverlay> coordinateAxesOverlay;

    SgInvariantGroupPtr gridGroup;

    struct GridInfo {
        double span;
        double interval;
        Vector3f color;
        bool isEnabled;
    };
    GridInfo gridInfos[3];

    LazyCaller updateGridsLater;

    double fps;
    Timer fpsTimer;
    int fpsCounter;
    Timer fpsRenderingTimer;
    bool fpsRendered;
    bool isDoingFpsTest;
    bool isFpsTestCanceled;

    MenuManager menuManager;
    Signal<void(SceneWidgetEvent* event, MenuManager* menu)> sigContextMenuRequest;

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
    void doFpsTest(int iteration);
    void onFpsUpdateRequest();
    void onFpsRenderingRequest();
    void renderFps();

    void onCurrentCameraChanged();
    void setVisiblePolygonElements(int elementFlags);
    int visiblePolygonElements() const;
    void setCollisionLineVisibility(bool on);

    void resetCursor();
    void setEditMode(bool on, bool doAdvertise);
    void toggleEditMode();
    void advertiseSceneModeChange(bool doModeSyncRequest);
    void advertiseSceneModeChangeInSubTree(SgNode* node);
    void activateCustomMode(SceneWidgetEventHandler* modeHandler, int modeId);
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
    EditableNodeInfo applyEditableFunction(
        vector<EditableNodeInfo>& editablePath,
        std::function<bool(SgNode* node, SceneWidgetEventHandler* handler)> function);
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

    void setScreenSize(int width, int height);
    void updateIndicator(const std::string& text);
    bool storeState(Archive& archive);
    void writeCameraPath(Mapping& archive, const std::string& key, int cameraIndex);
    Mapping* storeCameraState(
        int cameraIndex, SgCamera* camera, bool isBuiltinCamera, SgPosTransform* cameraTransform);
    bool restoreState(const Archive& archive);
    bool restoreCameraStates(const Listing& cameraListing, bool isSecondTrial);
    int readCameraPath(const Mapping& archive, const char* key);
    void restoreCurrentCamera(const Mapping& cameraData);

    void activateSystemNode(SgNode* node, bool on);
};

}


void SceneWidget::initializeClass(ExtensionManager* ext)
{
    auto glConfig = AppConfig::archive()->openMapping("OpenGL");
    
    isVerticalSyncMode_ = glConfig->get("vsync", false);

    isLowMemoryConsumptionMode_ = glConfig->get("low_memory_consumption", false);
    if(isLowMemoryConsumptionMode_){
        setLowMemoryConsumptionMode(true);
    }
}


void SceneWidget::setVerticalSyncMode(bool on)
{
    /*
      When the menu check item on the OpenGL vertical sync is toggled, the state is
      just saved into the config file and the application must be restarted to
      update the vsync state because it is impossible to change the state of the existing
      QOpenGLWidgets.
    */
    isVerticalSyncMode_ = on;
    AppConfig::archive()->openMapping("OpenGL")->write("vsync", on);
}


bool SceneWidget::isVerticalSyncMode()
{
    return isVerticalSyncMode_;
}


void SceneWidget::setLowMemoryConsumptionMode(bool on)
{
    isLowMemoryConsumptionMode_ = on;
    sigLowMemoryConsumptionModeChanged(on);

    auto glConfig = AppConfig::archive()->openMapping("OpenGL");
    if(on){
        glConfig->write("low_memory_consumption", true);
    } else {
        glConfig->remove("low_memory_consumption");
    }
}


bool SceneWidget::isLowMemoryConsumptionMode()
{
    return isLowMemoryConsumptionMode_;
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
        auto& path = paths[i];
        SceneWidgetRoot* root = static_cast<SceneWidgetRoot*>(path.front().get());
        function(root->sceneWidget(), path);
    }
}


SceneWidget::SceneWidget(QWidget* parent)
    : Widget(parent)
{
    impl = new Impl(this);

    sigSceneWidgetCreated_(this);
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
    if(glslRenderer){
        glslRenderer->setLowMemoryConsumptionMode(isLowMemoryConsumptionMode_);
        gl1Renderer = nullptr;
    } else {
        gl1Renderer = dynamic_cast<GL1SceneRenderer*>(renderer);
    }
        
    renderer->setOutputStream(MessageView::instance()->cout(false));
    renderer->enableUnusedResourceCheck(true);
    renderer->sigCurrentCameraChanged().connect([&](){ onCurrentCameraChanged(); });
    renderer->setCurrentCameraAutoRestorationMode(true);
    self->sigObjectNameChanged().connect([this](string name){ renderer->setName(name); });

    lastDevicePixelRatio = 1.0f;

    sceneRoot->sigUpdated().connect([this](const SgUpdate& update){ onSceneGraphUpdated(update); });
    tmpEditableArrays.emplace_back();
    pCurrentTmpEditableArray = tmpEditableArrays.begin();

    scene = renderer->scene();
    prevDefaultFramebufferObject = 0;
    isRendering = false;

    needToUpdatePreprocessedNodeTree = true;
    renderer->setFlagVariableToUpdatePreprocessedNodeTree(needToUpdatePreprocessedNodeTree);

    needToClearGLOnFrameBufferChange = false;

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
    builtinPersCamera->setName(N_("Perspective"));
    builtinPersCamera->setFieldOfView(radian(35.0));
    builtinCameraTransform->addChild(builtinPersCamera);

    builtinOrthoCamera = new SgOrthographicCamera;
    builtinOrthoCamera->setName(N_("Orthographic"));
    builtinOrthoCamera->setHeight(20.0f);
    builtinCameraTransform->addChild(builtinOrthoCamera);

    isBuiltinCameraCurrent = true;
    numBuiltinCameras = 2;
    systemGroup->addChild(builtinCameraTransform);
    isLightweightViewChangeEnabled = false;
    isCameraPositionInteractivelyChanged = false;
    isCameraRollRistricted = true;
    verticalAxis = 2; // Z
    timerToRenderNormallyAfterInteractiveCameraPositionChange.setSingleShot(true);
    timerToRenderNormallyAfterInteractiveCameraPositionChange.sigTimeout().connect(
        [&](){ tryToResumeNormalRendering(); });

    collisionLineVisibility = false;

    coordinateAxesOverlay = new CoordinateAxesOverlay;

    for(auto& info : gridInfos){
        info.span = 10.0;
        info.interval = 0.5;
        info.color << 0.9f, 0.9f, 0.9f;
        info.isEnabled = false;
    }
    gridInfos[XY_Grid].isEnabled = true;

    updateGrids();

    /*
    fpsTimer.sigTimeout().connect([&](){ onFpsUpdateRequest(); });
    fpsRenderingTimer.setSingleShot(true);
    fpsRenderingTimer.sigTimeout().connect([&](){ onFpsRenderingRequest(); });
    */

    isDoingFpsTest = false;

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
                sigModeSyncRequest_.connect(
                    [this](SceneWidget* requester){ impl->onModeSyncRequest(requester); });
            impl->setEditMode(isEditModeInModeSync, false);
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
    if(glslRenderer){
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


template<> GLSceneRenderer* SceneWidget::renderer<GLSceneRenderer>()
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
    
    lastDevicePixelRatio = devicePixelRatio();
    renderer->setDevicePixelRatio(lastDevicePixelRatio);

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
            latestEvent.type_ = SceneWidgetEvent::ModeChange;
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
                latestEvent.type_ = SceneWidgetEvent::PointerLeave;
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
            latestEvent.type_ = SceneWidgetEvent::FocusChange;
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
        renderer->updateViewportInformation();
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
        renderFps();
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


void SceneWidget::Impl::renderFps()
{
    /*
    renderer->setColor(Vector3f(1.0f, 1.0f, 1.0f));
    renderText(20, 20, QString("FPS: %1").arg(fps));
    */
    fpsRendered = true;
    ++fpsCounter;
}


void SceneWidget::Impl::onFpsUpdateRequest()
{
    double oldFps = fps;
    fps = fpsCounter / 0.5;
    fpsCounter = 0;
    fpsRendered = false;
    if(oldFps > 0.0 || fps > 0.0){
        fpsRenderingTimer.start(100);
    }
}
    

void SceneWidget::Impl::onFpsRenderingRequest()
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
    onFpsUpdateRequest();
}


void SceneWidget::doFpsTest(int iteration)
{
    impl->doFpsTest(iteration);
}
        

void SceneWidget::Impl::doFpsTest(int iteration)
{
    if(!isBuiltinCameraCurrent){
        showWarningDialog(
            _("FPS test cannot be executed because the current camera is not a built-in interactive camera."));
        return;
    }
    
    isDoingFpsTest = true;
    isFpsTestCanceled = false;
    
    const Vector3 p = lastClickedPoint;
    const Isometry3 C = builtinCameraTransform->T();

    QElapsedTimer timer;
    timer.start();

    int numFrames = 0;
    for(int i=0; i < iteration; ++i){
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
            if(isFpsTestCanceled){
                break;
            }
        }
    }

    double time = timer.elapsed() / 1000.0;
    fps = numFrames / time;
    fpsCounter = 0;

    update();

    QMessageBox::information(
        this, _("FPS Test Result"),
        QString(_("FPS: %1 frames / %2 [s] = %3")).arg(numFrames).arg(time).arg(fps));

    builtinCameraTransform->setTransform(C);
    update();

    isDoingFpsTest = false;
}


void SceneWidget::cancelFpsTest()
{
    impl->isFpsTestCanceled = true;
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
        if(!on || editModeBlockRequesters.empty()){
            isEditMode = on;
            resetCursor();
            sharedIndicatorLabel->clear();
            if(doAdvertise){
                advertiseSceneModeChange(true);
            }
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


void SceneWidget::blockEditMode(Referenced* requester)
{
    impl->editModeBlockRequesters.insert(requester);
    setEditMode(false);
}


void SceneWidget::unblockEditMode(Referenced* requester)
{
    impl->editModeBlockRequesters.erase(requester);
}


void SceneWidget::Impl::advertiseSceneModeChange(bool doModeSyncRequest)
{
    if(!isEditMode){
        // Clear focus
        latestEvent.type_ = SceneWidgetEvent::FocusChange;
        for(auto& editable : focusedEditablePath){
            editable.handler->onFocusChanged(&latestEvent, false);
        }
    }
    
    if(activeCustomModeHandler){
        latestEvent.type_ = SceneWidgetEvent::ModeChange;
        activeCustomModeHandler->onSceneModeChanged(&latestEvent);
    }
    
    advertiseSceneModeChangeInSubTree(sceneRoot);
    
    if(isEditMode){
        latestEvent.type_ = SceneWidgetEvent::FocusChange;
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
        sigModeSyncRequest_(self);
    }

    emitSigStateChangedLater();
}


void SceneWidget::Impl::advertiseSceneModeChangeInSubTree(SgNode* node)
{
    if(node->hasAttribute(SgObject::Operable)){
        if(auto editable = dynamic_cast<SceneWidgetEventHandler*>(node)){
            latestEvent.type_ = SceneWidgetEvent::ModeChange;
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
    impl->activateCustomMode(modeHandler, modeId);
}


void SceneWidget::Impl::activateCustomMode(SceneWidgetEventHandler* modeHandler, int modeId)
{
    auto prevHandler = activeCustomModeHandler;
    activeCustomModeHandler = modeHandler;
    activeCustomModeId = modeHandler ? modeId : 0;

    if(modeHandler != prevHandler){
        latestEvent.type_ = SceneWidgetEvent::ModeChange;
        resetCursor();
        if(prevHandler){
            prevHandler->onSceneModeChanged(&latestEvent);
        }
        if(modeHandler){
            modeHandler->onSceneModeChanged(&latestEvent);
        }
        sigStateChanged();
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
    if(!hasActiveInteractiveCamera()){
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
    if(glslRenderer){
        if(!pickingImageWindow){
            auto& vp = renderer->viewport();
            pickingImageWindow = new ImageWindow(vp.w, vp.h);
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
    switch(event->type()){
    case QEvent::KeyPress:
        latestEvent.type_ = SceneWidgetEvent::KeyPress;
        break;
    case QEvent::KeyRelease:
        latestEvent.type_ = SceneWidgetEvent::KeyRelease;
        break;
    default:
        latestEvent.type_ = SceneWidgetEvent::NoEvent;
    }
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
    switch(event->type()){
    case QEvent::MouseButtonPress:
        latestEvent.type_ = SceneWidgetEvent::ButtonPress;
        break;
    case QEvent::MouseButtonRelease:
        latestEvent.type_ = SceneWidgetEvent::ButtonRelease;
        break;
    case QEvent::MouseButtonDblClick:
        latestEvent.type_ = SceneWidgetEvent::DoubleClick;
        break;
    case QEvent::MouseMove:
        latestEvent.type_ = SceneWidgetEvent::PointerMove;
        break;
    default:
        latestEvent.type_ = SceneWidgetEvent::NoEvent;
    }
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

    const int r = lastDevicePixelRatio;
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
            if(auto editable = dynamic_cast<SceneWidgetEventHandler*>(node.get())){
                pointedEditablePath.emplace_back(node, editable);
            }
        }
    }
}


void SceneWidget::Impl::updateLastClickedPoint()
{
    auto& path = latestEvent.nodePath();
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


EditableNodeInfo SceneWidget::Impl::applyEditableFunction
(vector<EditableNodeInfo>& editablePath,
 std::function<bool(SgNode* node, SceneWidgetEventHandler* handler)> function)
{
    for(auto it = editablePath.rbegin(); it != editablePath.rend(); ++it){
        auto& editable = *it;
        if(function(editable.node, editable.handler)){
            return editable;
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

    latestEvent.type_ = SceneWidgetEvent::FocusChange;
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
        if(auto editable = dynamic_cast<SceneWidgetEventHandler*>(node.get())){
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
        auto info = applyEditableFunction(
            focusedEditablePath,
            [&](SgNode* node, SceneWidgetEventHandler* handler){
                bool handled = false;
                if(customNodeEventHandler){
                    handled = customNodeEventHandler(node, handler, &latestEvent);
                }
                if(!handled){
                    handled = handler->onKeyPressEvent(&latestEvent);
                }
                return handled;
            });
        if(info){
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
            latestEvent.button_ = Qt::MiddleButton;
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
            if(customNodeEventHandler){
                handled = customNodeEventHandler(
                    focusedEditable.node, focusedEditable.handler, &latestEvent);
            }
            if(!handled){
                handled = focusedEditable.handler->onKeyReleaseEvent(&latestEvent);
            }
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
                        latestEvent.type_ = SceneWidgetEvent::ButtonPress;
                        if(editable->onButtonPressEvent(&latestEvent)){
                            latestEvent.type_ = SceneWidgetEvent::ButtonRelease;
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
                    [&](SceneWidgetEventHandler* editable){
                        return editable->onPointerMoveEvent(&latestEvent);
                    });
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
                    latestEvent.type_ = SceneWidgetEvent::PointerLeave;
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
            string name = findObjectNameFromNodePath(latestEvent.nodePath());
            auto valueFormat = DisplayValueFormat::instance();
            string text;
            if(name.empty()){
                text = fmt::format("{0}: ({{0:.{1}f}} {{1:.{1}f}} {{2:.{1}f}})",
                                   _("Global Position"), valueFormat->lengthDecimals());
            } else {
                text = fmt::format("{0}: {1}, {2}: ({{0:.{3}f}} {{1:.{3}f}} {{2:.{3}f}})",
                                   _("Object"), name, _("Global Position"), valueFormat->lengthDecimals());
            }
            const Vector3& p = latestEvent.point();
            if(valueFormat->isMeter()){
                updateIndicator(fmt::format(text, p.x(), p.y(), p.z()));
            } else if(valueFormat->isMillimeter()){
                updateIndicator(fmt::format(text, p.x() * 1000.0, p.y() * 1000.0, p.z() * 100.0));
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
    latestEvent.type_ = SceneWidgetEvent::PointerLeave;
    
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

#if (QT_VERSION >= QT_VERSION_CHECK(5, 14, 0))
    const auto& pos = event->position();
    updateLatestEvent(pos.x(), pos.y(), event->modifiers());
#else
    updateLatestEvent(event->x(), event->y(), event->modifiers());
#endif

    latestEvent.type_ = SceneWidgetEvent::Scroll;
    updateLatestEventPath();
    updateLastClickedPoint();

    const int dy = event->angleDelta().y();
    const double s = dy / 8.0 / 15.0;
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

    if(!handled && hasActiveInteractiveCamera()){
        if(dy != 0){
            zoomView(0.25 * s);
        }
    }
}


bool SceneWidget::unproject(double x, double y, double z, Vector3& out_projected) const
{
    const int r = impl->lastDevicePixelRatio;
    return impl->renderer->unproject(r * x, r * y, z, out_projected);
}


SignalProxy<void(SceneWidget*)> SceneWidget::sigSceneWidgetCreated()
{
    return sigSceneWidgetCreated_;
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
    if(!isCameraRollRistricted){
        return T;
    }
    
    Vector3 up = Vector3::Unit(verticalAxis);
    Vector3 x, y;
    Vector3 z = T.linear().col(2).normalized();
    if(fabs(z.dot(up) > 0.9)){
        x = T.linear().col(0).normalized();
    } else {
        y = T.linear().col(1);
        if(y.dot(up) >= 0.0){
            x = up.cross(z).normalized();
        } else {
            x = z.cross(up).normalized();
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
    if(hasActiveInteractiveCamera()){

        switch(latestEvent.button()){
                
        case Qt::LeftButton:
            startViewRotation();
            break;
            
        case Qt::MiddleButton:
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

    if(!hasActiveInteractiveCamera()){
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

    if(!hasActiveInteractiveCamera()){
        dragMode = NO_DRAGGING;
        return;
    }
    
    const double dx = latestEvent.x() - orgMouseX;
    const double dy = latestEvent.y() - orgMouseY;

    Isometry3 R;
    if(isCameraRollRistricted){
        Vector3 up = Vector3::Unit(verticalAxis);
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

    if(!hasActiveInteractiveCamera()){
        dragMode = NO_DRAGGING;
        return;
    }

    const Isometry3& C = interactiveCameraTransform->T();

    if(isFirstPersonMode()){
        viewTranslationRatioX = -0.005;
        viewTranslationRatioY = -0.005;

    } else {
        auto& vp = renderer->viewport();
        const double aspect = static_cast<double>(vp.w) / vp.h;
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
        viewTranslationRatioX = r * cw / vp.w;
        viewTranslationRatioY = r * ch / vp.h;
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
    
    if(!hasActiveInteractiveCamera()){
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

    if(!hasActiveInteractiveCamera()){
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
    if(!hasActiveInteractiveCamera()){
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

    latestEvent.type_ = SceneWidgetEvent::ContextMenuRequest;
    latestEvent.contextMenu_ = &menuManager;

    bool modeMenuHandled = false;
    if(activeCustomModeHandler){
        modeMenuHandled = activeCustomModeHandler->onContextMenuRequest(&latestEvent);
    }

    bool nodeMenuHandled = false;
    if(!modeMenuHandled && !pointedEditablePath.empty()){
        auto editableToFocus = pointedEditablePath.front();
        for(auto p = pointedEditablePath.rbegin(); p != pointedEditablePath.rend(); ++p){
            auto& editableNode = *p;
            if(customNodeEventHandler){
                nodeMenuHandled = customNodeEventHandler(
                    editableNode.node, editableNode.handler, &latestEvent);
            }
            if(!nodeMenuHandled){
                editableNode.handler->onContextMenuRequest(&latestEvent);
            }
            int numItems = menuManager.numItems();
            if(numItems > prevNumItems){
                menuManager.addSeparator();
                prevNumItems = numItems;
                editableToFocus = editableNode;
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

    latestEvent.contextMenu_ = nullptr;
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


SignalProxy<void(SceneWidgetEvent* event, MenuManager* menu)>
SceneWidget::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}


void SceneWidget::overrideNodeEventHandler(NodeEventHandler handler)
{
    impl->customNodeEventHandler = handler;
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
    auto& path = impl->renderer->cameraPath(cameraIndex);
    for(size_t i=0; i < path.size() - 1; ++i){
        if(auto transform = dynamic_cast<InteractiveCameraTransform*>(path[i].get())){
            return transform;
        }
    }
    return nullptr;
}


InteractiveCameraTransform* SceneWidget::activeInteractiveCameraTransform()
{
    if(impl->hasActiveInteractiveCamera()){
        return impl->interactiveCameraTransform;
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
        auto& path = renderer->cameraPath(index);
        for(int i = path.size() - 2; i >= 0; --i){
            interactiveCameraTransform = dynamic_cast<InteractiveCameraTransform*>(path[i].get());
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


void SceneWidget::setHeadLightIntensity(double intensity)
{
    impl->renderer->headLight()->setIntensity(intensity);
}


void SceneWidget::setWorldLightIntensity(double /* intensity */)
{

}


void SceneWidget::setWorldLightAmbient(double intensity)
{
    impl->renderer->headLight()->setAmbientIntensity(intensity);
}


void SceneWidget::setGridEnabled(GridPlane plane, bool on)
{
    impl->gridInfos[plane].isEnabled = on;
}


bool SceneWidget::isGridEnabled(GridPlane plane) const
{
    return impl->gridInfos[plane].isEnabled;
}


void SceneWidget::setGridGeometry(GridPlane plane, double span, double interval)
{
    auto& info = impl->gridInfos[plane];
    info.span = span;
    info.interval = interval;
}


void SceneWidget::setGridColor(GridPlane plane, const Vector3f& color)
{
    impl->gridInfos[plane].color = color;
}


void SceneWidget::updateGrids()
{
    impl->updateGrids();
}


void SceneWidget::setFloorGridEnabled(bool on)
{
    setGridEnabled(XY_Grid, on);
    impl->updateGridsLater();
}


bool SceneWidget::isFloorGridEnabled() const
{
    return isGridEnabled(XY_Grid);
}


void SceneWidget::setFloorGridSpan(double span)
{
    impl->gridInfos[XY_Grid].span = span;
    impl->updateGridsLater();
}


void SceneWidget::setFloorGridInterval(double interval)
{
    impl->gridInfos[XY_Grid].interval = interval;
    impl->updateGridsLater();
}


void SceneWidget::setLineWidth(double width)
{
    impl->renderer->setDefaultLineWidth(width);
}


void SceneWidget::setPointSize(double size)
{
    impl->renderer->setDefaultPointSize(size);
}


void SceneWidget::setHeadLightEnabled(bool on)
{
    impl->renderer->headLight()->on(on);
}


void SceneWidget::setHeadLightLightingFromBack(bool on)
{
    if(impl->gl1Renderer){
        impl->gl1Renderer->setHeadLightLightingFromBackEnabled(on);
    }
}


void SceneWidget::setWorldLightEnabled(bool /* on */)
{

}


bool SceneWidget::isWorldLightEnabled() const
{
    return false;
}


void SceneWidget::setAdditionalLights(bool on)
{
    impl->renderer->enableAdditionalLights(on);
}


void SceneWidget::setCoordinateAxes(bool on)
{
    impl->activateSystemNode(impl->coordinateAxesOverlay, on);
}


void SceneWidget::setBackgroundColor(const Vector3& color)
{
    impl->renderer->setBackgroundColor(color.cast<float>());
    impl->update();
}


Vector3 SceneWidget::backgroundColor()
{
    return impl->renderer->backgroundColor().cast<double>();
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
    impl->builtinPersCamera->notifyUpdate(impl->sgUpdate.withAction(SgUpdate::Modified));
}


void SceneWidget::setClipDistances(double nearDistance, double farDistance)
{
    impl->builtinPersCamera->setNearClipDistance(nearDistance);
    impl->builtinPersCamera->setFarClipDistance(farDistance);
    impl->builtinOrthoCamera->setNearClipDistance(nearDistance);
    impl->builtinOrthoCamera->setFarClipDistance(farDistance);

    impl->sgUpdate.setAction(SgUpdate::Modified);
    impl->builtinPersCamera->notifyUpdate(impl->sgUpdate);
    impl->builtinOrthoCamera->notifyUpdate(impl->sgUpdate);
}


void SceneWidget::setInteractiveCameraRollRestricted(bool on)
{
    impl->isCameraRollRistricted = on;
}


void SceneWidget::setVerticalAxis(int axis)
{
    impl->verticalAxis = axis;
}


void SceneWidget::setLightweightViewChangeEnabled(bool on)
{
    impl->isLightweightViewChangeEnabled = on;
}


void SceneWidget::setHeight(double value)
{
    impl->builtinOrthoCamera->setHeight(value);
}


void SceneWidget::setNormalVisualization(bool on)
{
    impl->renderer->setNormalVisualizationEnabled(on);
}


void SceneWidget::setNormalLength(double length)
{
    impl->renderer->setNormalVisualizationLength(length);
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

    ListingPtr cameraListing = new Listing;
    set<SgPosTransform*> storedTransforms;
    int numCameras = renderer->numCameras();
    for(int i=0; i < numCameras; ++i){
        Mapping* cameraState = nullptr;
        auto camera = renderer->camera(i);
        if(self->isBuiltinCamera(camera)){
            auto transform = self->findOwnerInteractiveCameraTransform(i);
            if(transform && !storedTransforms.insert(transform).second){
                transform = nullptr; // already stored
            }
            cameraState = storeCameraState(i, camera, true, transform);
        } else if(i == renderer->currentCameraIndex()){
            cameraState = storeCameraState(i, camera, false, nullptr);
        }
        if(cameraState){
            cameraListing->append(cameraState);
        }
    }
    if(!cameraListing->empty()){
        archive.insert("cameras", cameraListing);
    }

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


Mapping* SceneWidget::Impl::storeCameraState
(int cameraIndex, SgCamera* camera, bool isBuiltinCamera, SgPosTransform* cameraTransform)
{
    auto state = new Mapping;
    writeCameraPath(*state, "camera", cameraIndex);

    if(cameraIndex == renderer->currentCameraIndex()){
        state->write("isCurrent", true);
    }

    if(isBuiltinCamera){
        if(auto ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
            state->write("orthoHeight", ortho->height());
        }
        if(cameraTransform){
            auto& T = cameraTransform->T();
            write(*state, "eye", T.translation());
            write(*state, "direction", SgCamera::direction(T));
            write(*state, "up", SgCamera::up(T));
        }
    }

    return state;
}


bool SceneWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool SceneWidget::Impl::restoreState(const Archive& archive)
{
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
            double height;
            if(cameraData.read("orthoHeight", height)){
                builtinOrthoCamera->setHeight(height);
            }
            builtinCameraTransform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));

            archive.addPostProcess([&](){ restoreCurrentCamera(cameraData); }, 1);
        }
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
                auto& cameraPath = renderer->cameraPath(cameraIndex);
                for(size_t j=0; j < cameraPath.size() - 1; ++j){
                    auto transform = dynamic_cast<SgPosTransform*>(cameraPath[j].get());
                    if(transform){
                        transform->setPosition(SgCamera::positionLookingFor(eye, direction, up));
                        transform->notifyUpdate(sgUpdate.withAction(SgUpdate::Modified));
                    }
                }
                updated = true;
            }
            SgCamera* camera = renderer->camera(cameraIndex);
            if(SgOrthographicCamera* ortho = dynamic_cast<SgOrthographicCamera*>(camera)){
                double height;
                if(state.read("orthoHeight", height)){
                    ortho->setHeight(height);
                    updated = true;
                }
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
        if(gridInfos[i].isEnabled){
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
    auto& info = gridInfos[index];

    grid->getOrCreateMaterial()->setDiffuseColor(info.color);

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
    float half = info.span / 2.0f;
    float interval = info.interval;
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
        systemGroup->addChildOnce(node, sgUpdate);
    } else {
        systemGroup->removeChild(node, sgUpdate);
    }
}
