/**
   @author Shin'ichiro Nakaoka
*/

#include "SceneWidget.h"
#include "GLSceneRenderer.h"
#include "SceneWidgetEditable.h"
#include "MainWindow.h"
#include "ToolBar.h"
#include "Dialog.h"
#include "SpinBox.h"
#include "Separator.h"
#include "Archive.h"
#include "MessageView.h"
#include "MenuManager.h"
#include "Timer.h"
#include "LazyCaller.h"
#include <cnoid/Selection>
#include <cnoid/EigenArchive>
#include <cnoid/SceneShape>
#include <cnoid/SceneCamera>
#include <cnoid/SceneLight>
#include <QGLWidget>
#include <QGLPixelBuffer>
#include <QLabel>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QFileDialog>
#include <QDialogButtonBox>
#include <QColorDialog>
#include <QElapsedTimer>
#include <QMessageBox>
#include <boost/bind.hpp>
#include <boost/format.hpp>
#include <set>
#include <iostream>
#include "gettext.h"

#ifdef _WIN32
#undef near
#undef far
#endif

using namespace std;
using namespace boost;
using namespace Eigen;
using namespace cnoid;

namespace {

const bool TRACE_FUNCTIONS = false;
const bool SHOW_IMAGE_FOR_PICKING = false;

class EditableExtractor : public SceneVisitor
{
public:
    vector<SgNode*> editables;

    int numEditables() const { return editables.size(); }
    SgNode* editableNode(int index) { return editables[index]; }
    SceneWidgetEditable* editable(int index) { return dynamic_cast<SceneWidgetEditable*>(editables[index]); }

    void apply(SgNode* node) {
        editables.clear();
        node->accept(*this);
    }

    virtual void visitNode(SgNode* node) {  }
    virtual void visitGroup(SgGroup* group) {
        if(SceneWidgetEditable* editable = dynamic_cast<SceneWidgetEditable*>(group)){
            editables.push_back(group);
        }
        for(SgGroup::const_iterator p = group->begin(); p != group->end(); ++p){
            (*p)->accept(*this);
        }
    }
    virtual void visitInvariantGroup(SgInvariantGroup* group) { }
    virtual void visitPosTransform(SgPosTransform* transform) {
        EditableExtractor::visitGroup(transform);
    }
    virtual void visitScaleTransform(SgScaleTransform* transform) {
        EditableExtractor::visitGroup(transform);
    }
    virtual void visitShape(SgShape* shape) {  }
    virtual void visitPlot(SgPlot* plot)  {  }
    virtual void visitPointSet(SgPointSet* pointSet) { }        
    virtual void visitLineSet(SgLineSet* lineSet) { }        
    virtual void visitPreprocessed(SgPreprocessed* preprocessed) {  }
    virtual void visitLight(SgLight* light) {  }
    virtual void visitCamera(SgCamera* camera) {  }
    virtual void visitOverlay(SgOverlay* overlay) { }
};

    
class SetupDialog : public Dialog
{
public:
    QVBoxLayout* vbox;
    SpinBox fieldOfViewSpin;
    DoubleSpinBox zNearSpin;
    DoubleSpinBox zFarSpin;
    CheckBox lightingCheck;
    CheckBox smoothShadingCheck;
    CheckBox headLightCheck;
    DoubleSpinBox headLightIntensitySpin;
    CheckBox headLightFromBackCheck;
    CheckBox worldLightCheck;
    DoubleSpinBox worldLightIntensitySpin;
    DoubleSpinBox worldLightAmbientSpin;
    CheckBox additionalLightsCheck;
    CheckBox floorGridCheck;
    DoubleSpinBox floorGridSpanSpin;
    DoubleSpinBox floorGridIntervalSpin;
    PushButton backgroundColorButton;
    PushButton gridColorButton;
    CheckBox textureCheck;
    PushButton defaultColorButton;
    DoubleSpinBox pointSizeSpin;
    DoubleSpinBox lineWidthSpin;
    CheckBox pointRenderingModeCheck;
    signals::connection pointRenderingModeCheckConnection;
    CheckBox normalVisualizationCheck;
    DoubleSpinBox normalLengthSpin;
    CheckBox coordinateAxesCheck;
    CheckBox fpsCheck;
    PushButton fpsTestButton;
    CheckBox bufferForPickingCheck;

    LazyCaller updateDefaultLightsLater;

    SetupDialog(SceneWidgetImpl* impl);
    void storeState(Archive& archive);
    void restoreState(const Archive& archive);
};


/**
   \note Z axis should always be the upper vertical direciton.
*/
Affine3 normalizedCameraTransform(const Affine3& T)
{
    Vector3 x, y;
    Vector3 z = T.linear().col(2).normalized();
        
    if(fabs(z.dot(Vector3::UnitZ())) > 0.9){
        x = T.linear().col(0).normalized();
        y = z.cross(x);
    } else {
        y = T.linear().col(1);
        if(y.dot(Vector3::UnitZ()) >= 0.0){
            x = Vector3::UnitZ().cross(z).normalized();
            y = z.cross(x);
        } else {
            x = z.cross(Vector3::UnitZ()).normalized();
            y = z.cross(x);
        }
    }
    Affine3 N;
    N.linear() << x, y, z;
    N.translation() = T.translation();
    return N;
}

boost::signal<void(SceneWidget*)> sigSceneWidgetCreated;

}


namespace cnoid {

class SceneWidgetImpl : public QGLWidget
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SceneWidgetImpl(SceneWidget* self);
    ~SceneWidgetImpl();

    SceneWidget* self;
    ostream& os;

    SceneWidgetRootPtr sceneRoot;
    GLSceneRenderer renderer;
    QGLPixelBuffer* buffer;
    LazyCaller initializeRenderingLater;
    SgUpdate modified;
    SgUpdate added;
    SgUpdate removed;

    SgPosTransformPtr builtinCameraTransform;
    SgPerspectiveCameraPtr builtinPersCamera;
    SgOrthographicCameraPtr builtinOrthoCamera;
    int numBuiltinCameras;
    bool isBuiltinCameraCurrent;
        
    SgDirectionalLightPtr worldLight;

    bool isEditMode;
    boost::signal<void(bool)> sigEditModeToggled;

    Selection viewpointControlMode;
    boost::signal<void(int mode)> sigViewpointControlModeChanged;
    bool isFirstPersionMode() const { return (viewpointControlMode.which() != SceneWidget::THIRD_PERSON_MODE); }
        
    enum DragMode { NO_DRAGGING, EDITING, VIEW_ROTATION, VIEW_TRANSLATION, VIEW_ZOOM } dragMode;

    set<SceneWidgetEditable*> editableEntities;

    typedef vector<SceneWidgetEditable*> EditablePath;
    EditablePath pointedEditablePath;
    SceneWidgetEditable* lastMouseMovedEditable;
    EditablePath focusedEditablePath;
    SceneWidgetEditable* focusedEditable;

    QCursor defaultCursor;

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

    SgGroupPtr systemNodeGroup;

    Selection polygonMode;
    bool collisionLinesVisible;

    SgCustomGLNodePtr coordinateAxes;
    SgPosTransform* xAxis;
    SgPosTransform* yAxis;
    SgPosTransform* zAxis;

    SgCustomGLNodePtr floorGrid;
    Vector4f gridColor;

    double fps;
    Timer fpsTimer;
    int fpsCounter;
    Timer fpsRenderingTimer;
    bool fpsRendered;

    boost::signal<void()> sigCamerasChanged;
    boost::signal<void(int)> sigCurrentCameraChanged;
        
    SetupDialog* setup;
    QLabel* indicatorLabel;

    MenuManager menuManager;
    boost::signal<void(const SceneWidgetEvent& event, MenuManager& menuManager)> sigContextMenuRequest;

    boost::signal<void(bool isFocused)> sigWidgetFocusChanged;
    boost::signal<void()> sigAboutToBeDestroyed;

    virtual void initializeGL();
    virtual void resizeGL(int width, int height);
    virtual void paintGL();

    void renderFloorGrid(GLSceneRenderer& renderer);
    void setupCoordinateAxes();
    void renderCoordinateAxes(GLSceneRenderer& renderer);

    void onSceneGraphUpdated(const SgUpdate& update);

    void showFPS(bool on);
    void doFPSTest();
    void onFPSUpdateRequest();
    void onFPSRenderingRequest();
    void renderFPS();

    void showBackgroundColorDialog();
    void showGridColorDialog();
    void showDefaultColorDialog();

    void updateCurrentCamera();
    void setCurrentCameraPath(std::vector<std::string>& simplifiedPathStrings);
    void onCamerasChanged();
    void onCurrentCameraChanged();

    void onTextureToggled(bool on);
    void onLineWidthChanged(double width);
    void onPointSizeChanged(double width);
    void setPolygonMode(int mode);
    void onPointRenderingModeToggled(bool on);
    void setCollisionLinesVisible(bool on);
    void onFieldOfViewChanged();
    void onClippingDepthChanged();
    void onLightingToggled(bool on);
    void onSmoothShadingToggled(bool on);
    void updateDefaultLights();
    void onNormalVisualizationChanged();

    void setEditMode(bool on);
    void toggleEditMode();
    void viewAll();

    void onEntityAdded(SgNode* node);
    void onEntityRemoved(SgNode* node);

    void onBufferForPickingToggled(bool on);
        
    void updateLatestEvent(QKeyEvent* event);
    void updateLatestEvent(int x, int y, int modifiers);
    void updateLatestEvent(QMouseEvent* event);
    bool updateLatestEventPath();
    void updateLastClickedPoint();
        
    SceneWidgetEditable* applyFunction(
        EditablePath& editablePath, function<bool(SceneWidgetEditable* editable)> function);
    bool setFocusToEditablePath(EditablePath& editablePath);
    bool setFocusToPointedEditablePath(SceneWidgetEditable* targetEditable);

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

    void startViewChange();
    void startViewRotation();
    void dragViewRotation();
    void startViewTranslation();
    void dragViewTranslation();
    void startViewZoom();
    void dragViewZoom();
    void zoomView(double ratio);

    void rotateBuiltinCameraView(double dPitch, double dYaw);
    void translateBuiltinCameraView(const Vector3& dp_local);

    void showViewModePopupMenu(const QPoint& globalPos);
    void showEditModePopupMenu(const QPoint& globalPos);

    void activateSystemNode(SgNodePtr node, bool on);

    bool saveImage(const std::string& filename);
    void setScreenSize(int width, int height);
    void updateIndicator(const std::string& text);
    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);
    void restoreCurrentCamera(const Mapping& cameraData);
};
}


SceneWidgetRoot::SceneWidgetRoot(SceneWidget* sceneWidget)
    : sceneWidget_(sceneWidget)
{

}


namespace {
void extractPathsFromSceneWidgetRoot(SgNode* node, SgNodePath& reversedPath, vector<SgNodePath>& paths)
{
    reversedPath.push_back(node);
    if(!node->hasOwners()){
        SceneWidgetRoot* sceneWidgetRoot = dynamic_cast<SceneWidgetRoot*>(node);
        if(sceneWidgetRoot){
            paths.push_back(reversedPath);
            std::reverse(paths.back().begin(), paths.back().end());
        }
    } else {
        SgObject::const_ownerIter p;
        for(p = node->ownerBegin(); p != node->ownerEnd(); ++p){
            SgNode* node = dynamic_cast<SgNode*>(*p);
            if(node){
                extractPathsFromSceneWidgetRoot(node, reversedPath, paths);
            }
        }
    }
    reversedPath.pop_back();
}
}
    

void SceneWidget::forEachInstance(SgNode* node, boost::function<void(SceneWidget* sceneWidget, const SgNodePath& path)> function)
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

    QVBoxLayout* vbox = new QVBoxLayout();
    vbox->setContentsMargins(0, 0, 0, 0);
    vbox->addWidget(impl);
    setLayout(vbox);

    ::sigSceneWidgetCreated(this);
}


SceneWidgetImpl::SceneWidgetImpl(SceneWidget* self)
    : QGLWidget(self),
      self(self),
      os(MessageView::mainInstance()->cout()),
      sceneRoot(new SceneWidgetRoot(self)),
      renderer(sceneRoot)
{
    setFocusPolicy(Qt::WheelFocus);

    setAutoBufferSwap(true);

    renderer.enableUnusedCacheCheck(true);
    renderer.sigRenderingRequest().connect(bind(&SceneWidgetImpl::update, this));
    renderer.sigCamerasChanged().connect(bind(&SceneWidgetImpl::onCamerasChanged, this));
    renderer.sigCurrentCameraChanged().connect(bind(&SceneWidgetImpl::onCurrentCameraChanged, this));

    sceneRoot->sigUpdated().connect(bind(&SceneWidgetImpl::onSceneGraphUpdated, this, _1));

    buffer = 0;

    initializeRenderingLater.setFunction(bind(&GLSceneRenderer::initializeRendering, &renderer));

    modified.setAction(SgUpdate::MODIFIED);
    added.setAction(SgUpdate::ADDED);
    removed.setAction(SgUpdate::REMOVED);

    gridColor << 0.9f, 0.9f, 0.9f, 1.0f;

    setAutoFillBackground(false);
    setMouseTracking(true);

    isEditMode = false;
    viewpointControlMode.resize(2);
    viewpointControlMode.setSymbol(SceneWidget::THIRD_PERSON_MODE, "thirdPerson");
    viewpointControlMode.setSymbol(SceneWidget::FIRST_PERSON_MODE, "firstPerson");
    viewpointControlMode.select(SceneWidget::THIRD_PERSON_MODE);
    dragMode = NO_DRAGGING;
    defaultCursor = self->cursor();

    lastMouseMovedEditable = 0;
    focusedEditable = 0;

    latestEvent.sceneWidget_ = self;

    lastClickedPoint.setZero();
    
    indicatorLabel = new QLabel();
    indicatorLabel->setAlignment(Qt::AlignLeft);
    QFont font = indicatorLabel->font();
    font.setFixedPitch(true);
    indicatorLabel->setFont(font);

    SgGroup* root = renderer.sceneRoot();

    builtinCameraTransform = new SgPosTransform();
    builtinCameraTransform->setTransform(
        SgCamera::positionLookingAt(
            Vector3(4.0, 2.0, 1.5), Vector3(0.0, 0.0, 1.0), Vector3::UnitZ()));

    builtinPersCamera = new SgPerspectiveCamera();
    builtinPersCamera->setName("Perspective");
    builtinPersCamera->setFieldOfView(0.6978);
    builtinCameraTransform->addChild(builtinPersCamera);

    builtinOrthoCamera = new SgOrthographicCamera();
    builtinOrthoCamera->setName("Orthographic");
    builtinOrthoCamera->setHeight(20.0f);
    builtinCameraTransform->addChild(builtinOrthoCamera);

    isBuiltinCameraCurrent = true;
    numBuiltinCameras = 2;
    root->addChild(builtinCameraTransform);

    setup = new SetupDialog(this);

    worldLight = new SgDirectionalLight();
    worldLight->setName("WorldLight");
    worldLight->setDirection(Vector3(0.0, 0.0, -1.0));
    root->addChild(worldLight);
    renderer.setAsDefaultLight(worldLight);

    updateDefaultLights();

    systemNodeGroup = new SgGroup();
    systemNodeGroup->setName("SystemGroup");
    root->addChild(systemNodeGroup);

    polygonMode.resize(3);
    polygonMode.setSymbol(SceneWidget::FILL_MODE, "fill");
    polygonMode.setSymbol(SceneWidget::LINE_MODE, "line");
    polygonMode.setSymbol(SceneWidget::POINT_MODE, "point");
    polygonMode.select(SceneWidget::FILL_MODE);

    collisionLinesVisible = false;

    setupCoordinateAxes();

    floorGrid = new SgCustomGLNode(bind(&SceneWidgetImpl::renderFloorGrid, this, _1));
    floorGrid->setName("FloorGrid");
    activateSystemNode(floorGrid, setup->floorGridCheck.isChecked());

    fpsTimer.sigTimeout().connect(bind(&SceneWidgetImpl::onFPSUpdateRequest, this));
    fpsRenderingTimer.setSingleShot(true);
    fpsRenderingTimer.sigTimeout().connect(bind(&SceneWidgetImpl::onFPSRenderingRequest, this));
}


SceneWidget::~SceneWidget()
{
    sigAboutToBeDestroyed();
    delete impl;
}


SceneWidgetImpl::~SceneWidgetImpl()
{
    if(buffer){
        buffer->makeCurrent();
        delete buffer;
    }
    delete indicatorLabel;
    delete setup;
}


SceneWidgetRoot* SceneWidget::sceneRoot()
{
    return impl->sceneRoot;
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

    if(!renderer.initializeGL()){
        os << "OpenGL initialization failed." << endl;
        // This view shoulbe be disabled when the glew initialization is failed.
    }
}


void SceneWidgetImpl::resizeGL(int width, int height)
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::resizeGL()" << endl;
    }

    renderer.setViewport(0, 0, width, height);
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
                    for(size_t i=0; i < focusedEditablePath.size(); ++i){
                        focusedEditablePath[i]->onFocusChanged(latestEvent, false);
                    }
                    focusedEditablePath.clear();
                    focusedEditable = 0;
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

    renderer.render();

    if(fpsTimer.isActive()){
        renderFPS();
    }
}


void SceneWidgetImpl::renderFloorGrid(GLSceneRenderer& renderer)
{
    if(!setup->floorGridCheck.isChecked()){
        return;
    }

    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    renderer.setColor(gridColor);

    float half = setup->floorGridSpanSpin.value() / 2.0f;
    float interval = setup->floorGridIntervalSpin.value();
    float i = 0.0f;
    float x = 0.0f;

    glBegin(GL_LINES);

    do {
        x = i * interval;
        // y-line
        glVertex3f( x, -half, 0.0f);
        glVertex3f( x,  half, 0.0f);
        glVertex3f(-x, -half, 0.0f);
        glVertex3f(-x,  half, 0.0f);
        // x-line
        glVertex3f(-half,  x, 0.0f);
        glVertex3f( half,  x, 0.0f);
        glVertex3f(-half, -x, 0.0f);
        glVertex3f( half, -x, 0.0f);
        ++i;
    } while(x < half);

    glEnd();

    glPopAttrib();
}


void SceneWidgetImpl::renderCoordinateAxes(GLSceneRenderer& renderer)
{
    glPushAttrib(GL_LIGHTING_BIT);
    glDisable(GL_LIGHTING);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    int x, y, w, h;
    renderer.getViewport(x, y, w, h);
    glOrtho((double)(x - 26), (double)(w - 26), (double)(y - 24), (double)(h - 24), -100.0, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    Affine3 transform(builtinCameraTransform->rotation());
    Affine3 inv = transform.inverse();
    glMultMatrixd(inv.data());

    renderer.setColor(Vector4f(1.0,0.0,0.0,0.0));
    renderer.visitPosTransform(xAxis);
    renderer.setColor(Vector4f(0.0,1.0,0.0,0.0));
    renderer.visitPosTransform(yAxis);
    renderer.setColor(Vector4f(0.4,0.6,1.0,0.0));
    renderer.visitPosTransform(zAxis);
    
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib();
}


void SceneWidgetImpl::renderFPS()
{
    renderer.setColor(Vector4f(1.0f, 1.0f, 1.0f, 1.0f));
    renderText(20, 20, QString("FPS: %1").arg(fps));
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


void SceneWidgetImpl::doFPSTest()
{
    const Vector3 p = lastClickedPoint;
    const Affine3 C = builtinCameraTransform->T();

    QElapsedTimer timer;
    timer.start();
    
    for(double i=1.0; i <= 360.0; i+=1.0){
        double a = 3.14159265 * i / 180.0;
        builtinCameraTransform->setTransform(
            normalizedCameraTransform(
                Translation3(p) *
                AngleAxis(a, Vector3::UnitZ()) *
                Translation3(-p) *
                C));
        glDraw();
    }

    double time = timer.elapsed() / 1000.0;
    fps = 360.0 / time;
    fpsCounter = 0;

    QMessageBox::information(setup, _("FPS Test Result"),
                             QString(_("FPS: %1 frames / %2 [s] = %3")).arg(360).arg(time).arg(fps));

    update();
}


void SceneWidget::setEditMode(bool on)
{
    impl->setEditMode(on);
}


bool SceneWidget::isEditMode() const
{
    return impl->isEditMode;
}


SignalProxy< boost::signal<void(bool)> > SceneWidget::sigEditModeToggled() const
{
    return impl->sigEditModeToggled;
}


void SceneWidgetImpl::setEditMode(bool on)
{
    if(on != isEditMode){
        isEditMode = on;
        sigEditModeToggled(on);

        set<SceneWidgetEditable*>::iterator p;
        for(p = editableEntities.begin(); p != editableEntities.end(); ++p){
            SceneWidgetEditable* editable = *p;
            editable->onSceneModeChanged(latestEvent);
        }

        if(!isEditMode){
            setCursor(defaultCursor);
        }
    }
}


void SceneWidgetImpl::toggleEditMode()
{
    setEditMode(!isEditMode);
}


void SceneWidget::setViewpointControlMode(ViewpointControlMode mode)
{
    impl->viewpointControlMode.select(mode);
    impl->sigViewpointControlModeChanged(mode);
}


SceneWidget::ViewpointControlMode SceneWidget::viewpointControlMode() const
{
    return static_cast<SceneWidget::ViewpointControlMode>(impl->viewpointControlMode.which());
}


SignalProxy< boost::signal<void(int mode)> > SceneWidget::sigViewpointControlModeChanged() const
{
    return impl->sigViewpointControlModeChanged;
}


void SceneWidget::viewAll()
{
    impl->viewAll();
}


void SceneWidgetImpl::viewAll()
{
    const BoundingBox& bbox = renderer.sceneRoot()->boundingBox();
    if(bbox.empty()){
        return;
    }
    const double radius = bbox.boundingSphereRadius();

    double left, right, bottom, top;
    renderer.getViewFrustum(*builtinPersCamera, left, right, bottom, top);
    
    Affine3& T = builtinCameraTransform->T();
    T.translation() +=
        (bbox.center() - T.translation())
        + T.rotation() * Vector3(0, 0, 2.0 * radius * builtinPersCamera->nearDistance() / (right - left));

    const double a = renderer.aspectRatio();
    if(a >= 1.0){
        builtinOrthoCamera->setHeight(radius * 2.0);
    } else {
        builtinOrthoCamera->setHeight(radius * 2.0 / a);
    }

    builtinOrthoCamera->notifyUpdate(modified);
}


void SceneWidgetImpl::onBufferForPickingToggled(bool on)
{
    if(!on){
        if(buffer){
            buffer->makeCurrent();
            delete buffer;
            buffer = 0;
        }
    }
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


bool SceneWidgetImpl::updateLatestEventPath()
{
    if(setup->bufferForPickingCheck.isChecked()){
        const QSize s = size();
        if(buffer && (buffer->size() != s)){
            buffer->makeCurrent();
            delete buffer;
            buffer = 0;
        }
        if(!buffer){
            if(QGLPixelBuffer::hasOpenGLPbuffers()){
                QGLFormat f = format();
                f.setDoubleBuffer(false);
                buffer = new QGLPixelBuffer(s, f, this);
                buffer->makeCurrent();
                glEnable(GL_DEPTH_TEST);
            }
        }
    }

    if(buffer){
        buffer->makeCurrent();
    } else {
        QGLWidget::makeCurrent();
    }

    bool picked = renderer.pick(latestEvent.x(), latestEvent.y());

    if(buffer){
        buffer->doneCurrent();
    } else if(SHOW_IMAGE_FOR_PICKING){
        swapBuffers();
    }

    latestEvent.nodePath_.clear();
    pointedEditablePath.clear();

    if(picked){
        latestEvent.point_ = renderer.pickedPoint();
        latestEvent.nodePath_ = renderer.pickedNodePath();

        SgNodePath& path = latestEvent.nodePath_;
        for(size_t i=0; i < path.size(); ++i){
            SceneWidgetEditable* editable = dynamic_cast<SceneWidgetEditable*>(path[i]);
            if(editable){
                pointedEditablePath.push_back(editable);
            }
        }
    }

    return picked;
}


void SceneWidgetImpl::updateLastClickedPoint()
{
    const SgNodePath& path = latestEvent.nodePath();
    if(!path.empty()){
        if(path.back() != floorGrid.get()){
            lastClickedPoint = latestEvent.point();
        }
    }
}

    
/**
   \return The editable object with which the given function is actually applied (the function returns true.)
   If there are no functions which returns true, null object is returned.
*/
SceneWidgetEditable* SceneWidgetImpl::applyFunction
(EditablePath& editablePath, function<bool(SceneWidgetEditable* editable)> function)
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
    if(!targetEditable){
        return false;
    }
    EditablePath path;
    for(size_t i=0; i < pointedEditablePath.size(); ++i){
        SceneWidgetEditable* editable = pointedEditablePath[i];
        path.push_back(editable);
        if(editable == targetEditable){
            return setFocusToEditablePath(path);
        }
    }
    return false;
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
        handled = applyFunction(focusedEditablePath, bind(&SceneWidgetEditable::onKeyPressEvent, _1, ref(latestEvent)));
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
                    handled = applyFunction(focusedEditablePath, bind(&SceneWidgetEditable::onRedoRequest, _1));
                } else {
                    handled = applyFunction(focusedEditablePath, bind(&SceneWidgetEditable::onUndoRequest, _1));
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
            if(setFocusToPointedEditablePath(
                   applyFunction(
                       pointedEditablePath, bind(&SceneWidgetEditable::onButtonPressEvent, _1, ref(latestEvent))))){
                handled = true;
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
        handled = setFocusToPointedEditablePath(
            applyFunction(
                pointedEditablePath, bind(&SceneWidgetEditable::onDoubleClickEvent, _1, ref(latestEvent))));
    }
    if(!handled){
        toggleEditMode();
    }
}


void SceneWidgetImpl::mouseReleaseEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    if(isEditMode){
        if(focusedEditable){
            updateLatestEventPath();
            focusedEditable->onButtonReleaseEvent(latestEvent);
        }
    }
    
    dragMode = NO_DRAGGING;
}


void SceneWidgetImpl::mouseMoveEvent(QMouseEvent* event)
{
    updateLatestEvent(event);

    switch(dragMode){

    case EDITING:
        focusedEditable->onPointerMoveEvent(latestEvent);
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


void SceneWidgetImpl::updatePointerPosition()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::updatePointerPosition()" << endl;
    }

    updateLatestEventPath();
    
    if(!isEditMode){
        static boost::format f(_("Position = (%.3f %.3f %.3f)"));
        const Vector3& p = latestEvent.point();
        updateIndicator(str(f % p.x() % p.y() % p.z()));
    } else {
        SceneWidgetEditable* mouseMovedEditable = applyFunction(
            pointedEditablePath, bind(&SceneWidgetEditable::onPointerMoveEvent, _1, ref(latestEvent)));

        if(mouseMovedEditable){
            if(!QWidget::hasFocus()){
                QWidget::setFocus(Qt::MouseFocusReason);
            }
        }
        if(lastMouseMovedEditable != mouseMovedEditable){
            if(!mouseMovedEditable){
                setCursor(defaultCursor);
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
        handled = setFocusToPointedEditablePath(
            applyFunction(
                pointedEditablePath, bind(&SceneWidgetEditable::onScrollEvent, _1, ref(latestEvent))));
    }    

    if(isBuiltinCameraCurrent){
        if(!handled || !isEditMode){
            if(event->orientation() == Qt::Vertical){
                zoomView(0.25 * s);
            }
        }
    }
}


const Affine3& SceneWidget::viewMatrix() const
{
    return impl->renderer.lastViewMatrix();
}


const Array4i& SceneWidget::viewport() const
{
    return impl->renderer.viewport();
}


bool SceneWidget::unproject(double x, double y, double z, Vector3& out_projected) const
{
    const Array4i& vp = viewport();

    Vector4 p;
    p[0] = 2.0 * (x - vp[0]) / vp[2] - 1.0;
    p[1] = 2.0 * (y - vp[1]) / vp[3] - 1.0;
    p[2] = 2.0 * z - 1.0;
    p[3] = 1.0;
    
    const Vector4 projected = (impl->renderer.lastProjectionMatrix() * viewMatrix().matrix()).inverse() * p;

    if(projected[3] == 0.0){
        return false;
    }

    out_projected.x() = projected.x() / projected[3];
    out_projected.y() = projected.y() / projected[3];
    out_projected.z() = projected.z() / projected[3];

    return true;
}


void SceneWidget::setCursor(const QCursor cursor)
{
    impl->setCursor(cursor);
}


SignalProxy< boost::signal<void(SceneWidget*)> > SceneWidget::sigSceneWidgetCreated()
{
    return ::sigSceneWidgetCreated;
}


SignalProxy< boost::signal<void(bool isFocused)> > SceneWidget::sigWidgetFocusChanged()
{
    return impl->sigWidgetFocusChanged;
}


void SceneWidgetImpl::focusInEvent(QFocusEvent* event)
{
    sigWidgetFocusChanged(true);
}


void SceneWidgetImpl::focusOutEvent(QFocusEvent* event)
{
    sigWidgetFocusChanged(false);
}


SignalProxy< boost::signal<void()> > SceneWidget::sigAboutToBeDestroyed()
{
    return impl->sigAboutToBeDestroyed;
}


void SceneWidgetImpl::startViewChange()
{
    if(isBuiltinCameraCurrent){

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
    
    orgMouseX = latestEvent.x();
    orgMouseY = latestEvent.y();
    orgCameraPosition = builtinCameraTransform->T();

    if(isFirstPersionMode()){
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
    
    const double dx = latestEvent.x() - orgMouseX;
    const double dy = latestEvent.y() - orgMouseY;
    const Vector3 right = SgCamera::right(orgCameraPosition);

    builtinCameraTransform->setTransform(
        normalizedCameraTransform(
            Translation3(orgPointedPos) *
            AngleAxis(-dx * dragAngleRatio, Vector3::UnitZ()) *
            AngleAxis(dy * dragAngleRatio, right) *
            Translation3(-orgPointedPos) *
            orgCameraPosition));
    builtinCameraTransform->notifyUpdate(modified);
}


void SceneWidgetImpl::startViewTranslation()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::startViewTranslation()" << endl;
    }

    const Affine3& C = builtinCameraTransform->T();

    if(isFirstPersionMode()){
        viewTranslationRatioX = -0.005;
        viewTranslationRatioY = -0.005;

    } else {
        int x, y, width, height;
        renderer.getViewport(x, y, width, height);
        const double aspect = (double)width / height;
        double r, cw, ch;
        SgCamera* camera = renderer.currentCamera();
        if(camera == builtinPersCamera){
            const double fovy = builtinPersCamera->fovy(aspect);
            r = (lastClickedPoint - C.translation()).dot(SgCamera::direction(C));
            ch = tanf(fovy / 2.0) * 2.0;
            cw = aspect * ch;
        } else if(camera == builtinOrthoCamera) {
            r = 1.0;
            ch = builtinOrthoCamera->height();
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
    
    const double dx = viewTranslationRatioX * (latestEvent.x() - orgMouseX);
    const double dy = viewTranslationRatioY * (latestEvent.y() - orgMouseY);

    builtinCameraTransform->setTransform(
        normalizedCameraTransform(
            Translation3(-dy * SgCamera::up(orgCameraPosition)) *
            Translation3(-dx * SgCamera::right(orgCameraPosition)) *
            orgCameraPosition));
    builtinCameraTransform->notifyUpdate(modified);
}


void SceneWidgetImpl::startViewZoom()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::startViewZoom()" << endl;
    }

    orgMouseY = latestEvent.y();
    orgCameraPosition = builtinCameraTransform->T();
    orgOrthoCameraHeight = builtinOrthoCamera->height();
    dragMode = VIEW_ZOOM;
}


void SceneWidgetImpl::dragViewZoom()
{
    if(TRACE_FUNCTIONS){
        os << "SceneWidgetImpl::dragViewZoom()" << endl;
    }

    SgCamera* camera = renderer.currentCamera();
    
    const double dy = latestEvent.y() - orgMouseY;
    const double ratio = expf(dy * 0.01);

    if(camera == builtinPersCamera){
        const Affine3& C = orgCameraPosition;
        const Vector3 v = SgCamera::direction(C);

        if(isFirstPersionMode()){
            double speed = 0.02;
            builtinCameraTransform->setTranslation(C.translation() + speed * dy * v);
            
        } else {
            const double l0 = (lastClickedPoint - C.translation()).dot(v);
            builtinCameraTransform->setTranslation(C.translation() + v * (l0 * (-ratio + 1.0)));
        }
        builtinCameraTransform->notifyUpdate(modified);

    } else if(camera == builtinOrthoCamera){
        builtinOrthoCamera->setHeight(orgOrthoCameraHeight * ratio);
        builtinOrthoCamera->notifyUpdate(modified);
    }
}


void SceneWidgetImpl::zoomView(double ratio)
{
    SgCamera* camera = renderer.currentCamera();
    if(camera == builtinPersCamera){
        const Affine3& C = builtinCameraTransform->T();
        const Vector3 v = SgCamera::direction(C);
        
        if(isFirstPersionMode()){
            if(latestEvent.modifiers() & Qt::ShiftModifier){
                ratio *= 5.0;
            }
            builtinCameraTransform->translation() += ratio * v;

        } else {
            if(latestEvent.modifiers() & Qt::ShiftModifier){
                ratio *= 0.2;
            }
            const double dz = ratio * (lastClickedPoint - C.translation()).dot(v);
            builtinCameraTransform->translation() -= dz * v;
        }
        builtinCameraTransform->notifyUpdate(modified);

    } else if(camera == builtinOrthoCamera){
        builtinOrthoCamera->setHeight(builtinOrthoCamera->height() * expf(ratio));
        builtinOrthoCamera->notifyUpdate(modified);
    }
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
        normalizedCameraTransform(
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
        normalizedCameraTransform(
            Translation3(T.linear() * dp_local) * T));
    builtinCameraTransform->notifyUpdate(modified);
}


void SceneWidgetImpl::showViewModePopupMenu(const QPoint& globalPos)
{
    menuManager.setNewPopupMenu(this);
    
    sigContextMenuRequest(latestEvent, menuManager);

    menuManager.setPath("/");
    menuManager.addItem(_("Edit Mode"))
        ->sigTriggered().connect(bind(&SceneWidgetImpl::toggleEditMode, this));

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

    sigContextMenuRequest(latestEvent, menuManager);
    if(menuManager.numItems() > prevNumItems){
        menuManager.addSeparator();
    }

    menuManager.setPath("/");
    menuManager.addItem(_("View Mode"))
        ->sigTriggered().connect(bind(&SceneWidgetImpl::toggleEditMode, this));
    
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


SignalProxy<boost::signal<void(const SceneWidgetEvent& event, MenuManager& menuManager)> >
SceneWidget::sigContextMenuRequest()
{
    return impl->sigContextMenuRequest;
}


void SceneWidgetImpl::showBackgroundColorDialog()
{
    const Vector3f& c = renderer.backgroundColor();
    QColor newColor =
        QColorDialog::getColor(
            QColor::fromRgbF(c[0], c[1], c[2], 1.0f),
            MainWindow::instance(), _("Background Color"));
    
    if(newColor.isValid()){
        renderer.setBackgroundColor(Vector3f(newColor.redF(), newColor.greenF(), newColor.blueF()));
        update();
    }
}


void SceneWidgetImpl::showGridColorDialog()
{
    QColor newColor = QColorDialog::getColor(
        QColor::fromRgbF(gridColor[0], gridColor[1], gridColor[2], gridColor[3]),
        MainWindow::instance(), _("Floor Grid Color"));
    
    if(newColor.isValid()){
        gridColor << newColor.redF(), newColor.greenF(), newColor.blueF(), newColor.alphaF();
        update();
    }
}


void SceneWidgetImpl::showDefaultColorDialog()
{
    QColor c = QColorDialog::getColor(
        QColor::fromRgbF(gridColor[0], gridColor[1], gridColor[2], gridColor[3]),
        MainWindow::instance(), _("Default Color"));
    
    if(c.isValid()){
        Vector4f color(c.redF(), c.greenF(), c.blueF(), c.alphaF());
        renderer.setDefaultColor(color);
        renderer.defaultMaterial()->setDiffuseColor(Vector3f(c.redF(), c.greenF(), c.blueF()));
        renderer.defaultMaterial()->setTransparency(1.0f - c.alphaF());
        renderer.requestToClearCache();
        update();
    }
}


SignalProxy<boost::signal<void()> > SceneWidget::sigCamerasChanged() const
{
    return impl->sigCamerasChanged;
}


SignalProxy<boost::signal<void(int)> > SceneWidget::sigCurrentCameraChanged() const
{
    return impl->sigCurrentCameraChanged;
}


int SceneWidget::numCameras() const
{
    return impl->renderer.numCameras();
}


bool SceneWidget::getSimplifiedCameraPathStrings(int index, std::vector<std::string>& pathStrings) const
{
    return impl->renderer.getSimplifiedCameraPathStrings(index, pathStrings);
}


int SceneWidget::currentCameraIndex() const
{
    return impl->renderer.currentCameraIndex();
}


void SceneWidgetImpl::updateCurrentCamera()
{
    const int index = renderer.currentCameraIndex();
    if(index >= 0){
        latestEvent.cameraPath_ = renderer.cameraPath(index);
        sigCurrentCameraChanged(renderer.currentCameraIndex());
    }
}
    
        
void SceneWidget::setCurrentCamera(int index)
{
    impl->renderer.setCurrentCamera(index);
    impl->updateCurrentCamera();
}


SgPosTransform* SceneWidget::builtinCameraTransform()
{
    return impl->builtinCameraTransform.get();
}


SgPerspectiveCamera* SceneWidget::builtinPerspectiveCamera() const
{
    return impl->builtinPersCamera.get();
}


SgOrthographicCamera* SceneWidget::builtinOrthographicCamera() const
{
    return impl->builtinOrthoCamera.get();
}


bool SceneWidget::isBuiltinCameraCurrent() const
{
    return impl->isBuiltinCameraCurrent;
}


void SceneWidgetImpl::setCurrentCameraPath(std::vector<std::string>& simplifiedPathStrings)
{
    renderer.setCurrentCamera(simplifiedPathStrings);
    updateCurrentCamera();
}


void SceneWidgetImpl::onCamerasChanged()
{
    sigCamerasChanged();
    updateCurrentCamera();
}


void SceneWidgetImpl::onCurrentCameraChanged()
{
    SgCamera* current = renderer.currentCamera();
    isBuiltinCameraCurrent = (current == builtinPersCamera || current == builtinOrthoCamera);
}


void SceneWidgetImpl::onTextureToggled(bool on)
{
    renderer.enableTexture(on);
    update();
}


void SceneWidgetImpl::onLineWidthChanged(double width)
{
    renderer.setDefaultLineWidth(width);
    update();
}


void SceneWidgetImpl::onPointSizeChanged(double size)
{
    renderer.setDefaultPointSize(size);
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
        setup->pointRenderingModeCheckConnection.block();
        setup->pointRenderingModeCheck.setChecked(true);
        setup->pointRenderingModeCheckConnection.unblock();
    }
    if(mode == SceneWidget::LINE_MODE && setup->pointRenderingModeCheck.isChecked()){
        polygonMode.select(SceneWidget::POINT_MODE);
    } else {
        polygonMode.select(mode);
    }

    if(polygonMode.which() != oldMode){
        switch(polygonMode.which()){
        case SceneWidget::FILL_MODE:
            renderer.setPolygonMode(GLSceneRenderer::FILL_MODE);
            break;
        case SceneWidget::LINE_MODE:
            renderer.setPolygonMode(GLSceneRenderer::LINE_MODE);
            break;
        case SceneWidget::POINT_MODE:
            renderer.setPolygonMode(GLSceneRenderer::POINT_MODE);
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
        renderer.property()->write("collision", on);
        update();
    }
}


bool SceneWidget::collisionLinesVisible() const
{
    return impl->collisionLinesVisible;
}


void SceneWidgetImpl::onFieldOfViewChanged()
{
    builtinPersCamera->setFieldOfView(PI * setup->fieldOfViewSpin.value() / 180.0);
    builtinPersCamera->notifyUpdate(modified);
}


void SceneWidgetImpl::onClippingDepthChanged()
{
    double zNear = setup->zNearSpin.value();
    double zFar = setup->zFarSpin.value();
    builtinPersCamera->setNearDistance(zNear);
    builtinPersCamera->setFarDistance(zFar);
    builtinOrthoCamera->setNearDistance(zNear);
    builtinOrthoCamera->setFarDistance(zFar);
    builtinOrthoCamera->notifyUpdate(modified);
    builtinPersCamera->notifyUpdate(modified);
}


void SceneWidgetImpl::onLightingToggled(bool on)
{
    renderer.setDefaultLighting(on);
    update();
}


void SceneWidgetImpl::onSmoothShadingToggled(bool on)
{
    renderer.setDefaultSmoothShading(on);
    update();
}


void SceneWidgetImpl::updateDefaultLights()
{
    SgLight* headLight = renderer.headLight();
    headLight->on(setup->headLightCheck.isChecked());
    headLight->setIntensity(setup->headLightIntensitySpin.value());
    renderer.setHeadLightLightingFromBackEnabled(setup->headLightFromBackCheck.isChecked());

    worldLight->on(setup->worldLightCheck.isChecked());
    worldLight->setIntensity(setup->worldLightIntensitySpin.value());
    worldLight->setAmbientIntensity(setup->worldLightAmbientSpin.value());

    renderer.enableAdditionalLights(setup->additionalLightsCheck.isChecked());

    worldLight->notifyUpdate(modified);
}


void SceneWidgetImpl::onNormalVisualizationChanged()
{
    if(setup->normalVisualizationCheck.isChecked()){
        renderer.showNormalVectors(setup->normalLengthSpin.value());
    } else {
        renderer.showNormalVectors(0.0);
    }
    update();
}


void SceneWidget::setHeadLightIntensity(double value)
{
    impl->setup->headLightIntensitySpin.setValue(value);
}


void SceneWidget::setWorldLightIntensity(double value)
{
    impl->setup->worldLightIntensitySpin.setValue(value);
}


void SceneWidget::setWorldLightAmbient(double value)
{
    impl->setup->worldLightAmbientSpin.setValue(value);
}


void SceneWidget::setFloorGridSpan(double value)
{
    impl->setup->floorGridSpanSpin.setValue(value);
}


void SceneWidget::setFloorGridInterval(double value)
{
    impl->setup->floorGridIntervalSpin.setValue(value);
}


void SceneWidget::setLineWidth(double value)
{
    impl->setup->lineWidthSpin.setValue(value);
}


void SceneWidget::setPointSize(double value)
{
    impl->setup->pointSizeSpin.setValue(value);
}


void SceneWidget::setNormalLength(double value)
{
    impl->setup->normalLengthSpin.setValue(value);
}


void SceneWidget::setHeadLight(bool on)
{
    impl->setup->headLightCheck.setChecked(on);
}


void SceneWidget::setHeadLightLightingFromBack(bool on)
{
    impl->setup->headLightFromBackCheck.setChecked(on);
}


void SceneWidget::setWorldLight(bool on)
{
    impl->setup->worldLightCheck.setChecked(on);
}


void SceneWidget::setAdditionalLights(bool on)
{
    impl->setup->additionalLightsCheck.setChecked(on);
}


void SceneWidget::setFloorGrid(bool on)
{
    impl->setup->floorGridCheck.setChecked(on);
}


void SceneWidget::setNormalVisualization(bool on)
{
    impl->setup->normalVisualizationCheck.setChecked(on);
}


void SceneWidget::setCoordinateAxes(bool on)
{
    impl->setup->coordinateAxesCheck.setChecked(on);
}


void SceneWidget::setUseBufferForPicking(bool on)
{
    impl->setup->bufferForPickingCheck.setChecked(on);
}


void SceneWidget::setBackgroundColor(const Vector3& color)
{
    impl->renderer.setBackgroundColor(color.cast<float>());
}


void SceneWidget::setColor(const Vector4& color)
{
    impl->renderer.setColor(color.cast<float>());
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
    impl->setup->zNearSpin.setValue(value);
}


void SceneWidget::setFar(double value)
{
    impl->setup->zFarSpin.setValue(value);
}


void SceneWidget::showSetupDialog()
{
    impl->setup->show();
}


QVBoxLayout* SceneWidget::setupDialogVBox()
{
    return impl->setup->vbox;
}


bool SceneWidget::saveImage(const std::string& filename)
{
    return impl->saveImage(filename);
}


bool SceneWidgetImpl::saveImage(const std::string& filename)
{
    return grabFrameBuffer().save(filename.c_str());
}


QImage SceneWidget::getImage()
{
    return impl->grabFrameBuffer();
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
    archive.write("viewpointControlMode", viewpointControlMode.selectedSymbol());
    archive.write("collisionLines", collisionLinesVisible);
    archive.write("polygonMode", polygonMode.selectedSymbol());

    setup->storeState(archive);

    Mapping& cameraData = *archive.createMapping("camera");

    vector<string> cameraStrings;
    if(renderer.getSimplifiedCameraPathStrings(renderer.currentCameraIndex(), cameraStrings)){
        if(cameraStrings.size() == 1){
            cameraData.write("current", cameraStrings.front());
        } else {
            Listing& pathNode = *cameraData.createListing("current");
            pathNode.setFlowStyle(true);
            for(size_t i=0; i < cameraStrings.size(); ++i){
                pathNode.append(cameraStrings[i]);
            }
        }
    }
    const Affine3& C = builtinCameraTransform->T();
    write(cameraData, "eye", C.translation());
    write(cameraData, "direction", SgCamera::direction(C));
    write(cameraData, "up", SgCamera::up(C));

    cameraData.write("fieldOfView", builtinPersCamera->fieldOfView());
    cameraData.write("near", setup->zNearSpin.value());
    cameraData.write("far", setup->zFarSpin.value());
    cameraData.write("orthoHeight", builtinOrthoCamera->height());

    write(archive, "backgroundColor", renderer.backgroundColor());
    write(archive, "gridColor", gridColor);
    
    return true;
}


bool SceneWidget::restoreState(const Archive& archive)
{
    return impl->restoreState(archive);
}


bool SceneWidgetImpl::restoreState(const Archive& archive)
{
    bool doUpdate = false;

    int oldViewpointControlMode = viewpointControlMode.which();
    string symbol;
    if(archive.read("viewpointControlMode", symbol)){
        viewpointControlMode.select(symbol);
        if(viewpointControlMode.which() != oldViewpointControlMode){
            sigViewpointControlModeChanged(viewpointControlMode.which());
        }
    }
    if(archive.read("polygonMode", symbol)){
        setPolygonMode(polygonMode.index(symbol));
    }

    setCollisionLinesVisible(archive.get("collisionLines", collisionLinesVisible));
    
    setup->restoreState(archive);
    
    const Mapping& cameraData = *archive.findMapping("camera");
    if(cameraData.isValid()){
        Vector3 eye, direction, up;
        if(read(cameraData, "eye", eye) &&
           read(cameraData, "direction", direction) &&
           read(cameraData, "up", up)){
            builtinCameraTransform->setPosition(SgCamera::positionLookingFor(eye, direction, up));
            doUpdate = true;
        }
        double fov;
        if(cameraData.read("fieldOfView", fov)){
            builtinPersCamera->setFieldOfView(fov);
            doUpdate = true;
        }
        double height;
        if(cameraData.read("orthoHeight", height)){
            builtinOrthoCamera->setHeight(height);
            doUpdate = true;
        }
        setup->zNearSpin.setValue(cameraData.get("near", static_cast<double>(builtinPersCamera->nearDistance())));
        setup->zFarSpin.setValue(cameraData.get("far", static_cast<double>(builtinPersCamera->farDistance())));
        
        archive.addPostProcess(bind(&SceneWidgetImpl::restoreCurrentCamera, this, ref(cameraData)));
    }

    Vector3f bgColor;
    if(read(archive, "backgroundColor", bgColor)){
        renderer.setBackgroundColor(bgColor);
        doUpdate = true;
    }
    if(read(archive, "gridColor", gridColor)){
        doUpdate = true;
    }
    if(doUpdate){
        update();
    }
    return true;
}


void SceneWidgetImpl::restoreCurrentCamera(const Mapping& cameraData)
{
    // get a path strings of the current camera
    vector<string> cameraStrings;
    const ValueNode& current = *cameraData.find("current");
    if(current.isString()){
        cameraStrings.push_back(current.toString());
    } else if(current.isListing()){
        const Listing& pathNode = *current.toListing();
        for(int i=0; i < pathNode.size(); ++i){
            cameraStrings.push_back(pathNode[i].toString());
        }
    }
    if(!cameraStrings.empty()){
        if(renderer.numCameras() == 0){
            renderer.initializeRendering();
        }
        setCurrentCameraPath(cameraStrings);
    }
}


void SceneWidgetImpl::setupCoordinateAxes()
{
    coordinateAxes = new SgCustomGLNode(bind(&SceneWidgetImpl::renderCoordinateAxes, this, _1));
    coordinateAxes->setName("CoordinateAxes");

    float length = 16;
    float width = 2;

    SgMeshPtr cylinder = new SgMesh();
    cylinder->setCylinder(width, length);
    SgMeshPtr cone = new SgMesh();
    cone->setCone(width * 2.0, width * 4.0);

    SgShapePtr xCylinderShape = new SgShape;
    xCylinderShape->setMesh(cylinder);
    SgShapePtr xConeShape = new SgShape;
    xConeShape->setMesh(cone);
    SgPosTransform* xtransform = new SgPosTransform;
    xtransform->setTranslation(Vector3(0.0, length / 2.0, 0.0));
    xtransform->addChild(xConeShape);
    xAxis = new SgPosTransform;
    xAxis->setTranslation(Vector3(length / 2.0, 0.0, 0.0));
    AngleAxis xangleA(1.571, Vector3(0.0, 0.0, -1.0));
    xAxis->setRotation(xangleA.toRotationMatrix ());
    xAxis->addChild(xCylinderShape);
    xAxis->addChild(xtransform);

    SgShapePtr yCylinderShape = new SgShape;
    yCylinderShape->setMesh(cylinder);
    SgShapePtr yConeShape = new SgShape;
    yConeShape->setMesh(cone);
    SgPosTransform* ytransform = new SgPosTransform;
    ytransform->setTranslation(Vector3(0.0, length/2.0, 0.0));
    ytransform->addChild(yConeShape);
    yAxis = new SgPosTransform;
    yAxis->setTranslation(Vector3(0.0, length / 2.0, 0.0));
    yAxis->addChild(yCylinderShape);
    yAxis->addChild(ytransform);

    SgShapePtr zCylinderShape = new SgShape;
    zCylinderShape->setMesh(cylinder);
    SgShapePtr zConeShape = new SgShape;
    zConeShape->setMesh(cone);
    SgPosTransform* ztransform = new SgPosTransform;
    ztransform->setTranslation(Vector3(0.0, length / 2.0, 0.0));
    ztransform->addChild(zConeShape);
    zAxis = new SgPosTransform;
    AngleAxis zangleA(1.571, Vector3(1.0, 0.0, 0.0));
    zAxis->setRotation(zangleA.toRotationMatrix ());
    zAxis->setTranslation(Vector3(0.0, 0.0, length / 2.0));
    zAxis->addChild(zCylinderShape);
    zAxis->addChild(ztransform);

    activateSystemNode(coordinateAxes, setup->coordinateAxesCheck.isChecked());
}


void SceneWidgetImpl::activateSystemNode(SgNodePtr node, bool on)
{
    if(on){
        systemNodeGroup->addChild(node, true);
    } else {
        systemNodeGroup->removeChild(node, true);
    }
}


SetupDialog::SetupDialog(SceneWidgetImpl* impl)
{
    setWindowTitle(_("SceneWidget Setup"));

    QVBoxLayout* topVBox = new QVBoxLayout();
    vbox = new QVBoxLayout();
    QHBoxLayout* hbox;
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Default Camera"))));
    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Field of view")));
    fieldOfViewSpin.setRange(1, 179);
    fieldOfViewSpin.setValue(45);
    fieldOfViewSpin.sigValueChanged().connect(bind(&SceneWidgetImpl::onFieldOfViewChanged, impl));
    hbox->addWidget(&fieldOfViewSpin);
    hbox->addWidget(new QLabel(_("[deg]")));
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    hbox->addWidget(new QLabel(_("Clipping depth")));
    hbox->addSpacing(8);
    hbox->addWidget(new QLabel(_("Near")));
    zNearSpin.setDecimals(4);
    zNearSpin.setRange(0.0001, 9.9999);
    zNearSpin.setSingleStep(0.0001);
    zNearSpin.setValue(impl->builtinPersCamera->nearDistance());
    zNearSpin.sigValueChanged().connect(bind(&SceneWidgetImpl::onClippingDepthChanged, impl));
    hbox->addWidget(&zNearSpin);
    hbox->addWidget(new QLabel(_("Far")));
    zFarSpin.setDecimals(1);
    zFarSpin.setRange(0.1, 9999999.9);
    zFarSpin.setSingleStep(0.1);
    zFarSpin.setValue(impl->builtinPersCamera->farDistance());
    zFarSpin.sigValueChanged().connect(bind(&SceneWidgetImpl::onClippingDepthChanged, impl));
    hbox->addWidget(&zFarSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);

    updateDefaultLightsLater.setFunction(bind(&SceneWidgetImpl::updateDefaultLights, impl));
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Light"))));
    hbox = new QHBoxLayout();
    lightingCheck.setText(_("Lighiting"));
    lightingCheck.setChecked(true);
    lightingCheck.sigToggled().connect(bind(&SceneWidgetImpl::onLightingToggled, impl, _1));
    hbox->addWidget(&lightingCheck);

    smoothShadingCheck.setText(_("Smooth shading"));
    smoothShadingCheck.setChecked(true);
    smoothShadingCheck.sigToggled().connect(bind(&SceneWidgetImpl::onSmoothShadingToggled, impl, _1));
    hbox->addWidget(&smoothShadingCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    headLightCheck.setText(_("Head light"));
    headLightCheck.setChecked(true);
    headLightCheck.sigToggled().connect(bind(updateDefaultLightsLater));
    hbox->addWidget(&headLightCheck);

    hbox->addWidget(new QLabel(_("Intensity")));
    headLightIntensitySpin.setDecimals(2);
    headLightIntensitySpin.setSingleStep(0.01);    
    headLightIntensitySpin.setRange(0.0, 1.0);
    headLightIntensitySpin.setValue(0.75);
    headLightIntensitySpin.sigValueChanged().connect(bind(updateDefaultLightsLater));
    hbox->addWidget(&headLightIntensitySpin);

    headLightFromBackCheck.setText(_("Back lighting"));
    headLightFromBackCheck.sigToggled().connect(bind(updateDefaultLightsLater));
    hbox->addWidget(&headLightFromBackCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    worldLightCheck.setText(_("World light"));
    worldLightCheck.setChecked(true);
    worldLightCheck.sigToggled().connect(bind(updateDefaultLightsLater));
    hbox->addWidget(&worldLightCheck);

    hbox->addWidget(new QLabel(_("Intensity")));
    worldLightIntensitySpin.setDecimals(2);
    worldLightIntensitySpin.setSingleStep(0.01);    
    worldLightIntensitySpin.setRange(0.0, 1.0);
    worldLightIntensitySpin.setValue(0.5);
    worldLightIntensitySpin.sigValueChanged().connect(bind(updateDefaultLightsLater));
    hbox->addWidget(&worldLightIntensitySpin);

    hbox->addWidget(new QLabel(_("Ambient")));
    worldLightAmbientSpin.setDecimals(2);
    worldLightAmbientSpin.setSingleStep(0.01);    
    worldLightAmbientSpin.setRange(0.0, 1.0);
    worldLightAmbientSpin.setValue(0.3);
    worldLightAmbientSpin.sigValueChanged().connect(bind(updateDefaultLightsLater));
    hbox->addWidget(&worldLightAmbientSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    additionalLightsCheck.setText(_("Additional lights"));
    additionalLightsCheck.setChecked(true);
    additionalLightsCheck.sigToggled().connect(bind(updateDefaultLightsLater));
    hbox->addWidget(&additionalLightsCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    vbox->addLayout(new HSeparatorBox(new QLabel(_("Background"))));
    hbox = new QHBoxLayout();
    backgroundColorButton.setText(_("Background color"));
    backgroundColorButton.sigClicked().connect(bind(&SceneWidgetImpl::showBackgroundColorDialog, impl));
    hbox->addWidget(&backgroundColorButton);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    floorGridCheck.setText(_("Show the floor grid"));
    floorGridCheck.setChecked(true);
    floorGridCheck.sigToggled().connect(
        bind(&SceneWidgetImpl::activateSystemNode, impl, boost::ref(impl->floorGrid), _1));
    hbox->addWidget(&floorGridCheck);
    
    hbox->addWidget(new QLabel(_("Span")));
    floorGridSpanSpin.setAlignment(Qt::AlignCenter);
    floorGridSpanSpin.setDecimals(1);
    floorGridSpanSpin.setRange(0.0, 99.9);
    floorGridSpanSpin.setSingleStep(0.1);
    floorGridSpanSpin.setValue(10.0);
    floorGridSpanSpin.sigValueChanged().connect(bind(&SceneWidgetImpl::update, impl));
    hbox->addWidget(&floorGridSpanSpin);
    hbox->addSpacing(8);
    
    hbox->addWidget(new QLabel(_("Interval")));
    floorGridIntervalSpin.setAlignment(Qt::AlignCenter);
    floorGridIntervalSpin.setDecimals(2);
    floorGridIntervalSpin.setRange(0.01, 9.99);
    floorGridIntervalSpin.setSingleStep(0.01);
    floorGridIntervalSpin.setValue(0.5);
    floorGridIntervalSpin.sigValueChanged().connect(bind(&SceneWidgetImpl::update, impl));
    hbox->addWidget(&floorGridIntervalSpin);

    gridColorButton.setText(_("Color"));
    gridColorButton.sigClicked().connect(bind(&SceneWidgetImpl::showGridColorDialog, impl));
    hbox->addWidget(&gridColorButton);
    hbox->addStretch();
    vbox->addLayout(hbox);

    vbox->addWidget(new HSeparator());
    hbox = new QHBoxLayout();
    textureCheck.setText(_("Texture"));
    textureCheck.setChecked(true);
    textureCheck.sigToggled().connect(bind(&SceneWidgetImpl::onTextureToggled, impl, _1));
    hbox->addWidget(&textureCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    defaultColorButton.setText(_("Default color"));
    defaultColorButton.sigClicked().connect(bind(&SceneWidgetImpl::showDefaultColorDialog, impl));
    hbox->addWidget(&defaultColorButton);
    
    hbox->addWidget(new QLabel(_("Default line width")));
    lineWidthSpin.setDecimals(1);
    lineWidthSpin.setRange(0.1, 9.9);
    lineWidthSpin.setSingleStep(0.1);
    lineWidthSpin.setValue(1.0);
    lineWidthSpin.sigValueChanged().connect(bind(&SceneWidgetImpl::onLineWidthChanged, impl, _1));
    hbox->addWidget(&lineWidthSpin);

    hbox->addWidget(new QLabel(_("Default point size")));
    pointSizeSpin.setDecimals(1);
    pointSizeSpin.setRange(0.1, 9.9);
    pointSizeSpin.setSingleStep(0.1);
    pointSizeSpin.setValue(1.0);
    pointSizeSpin.sigValueChanged().connect(bind(&SceneWidgetImpl::onPointSizeChanged, impl, _1));
    hbox->addWidget(&pointSizeSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    pointRenderingModeCheck.setText(_("Do point rendering in the wireframe mode"));
    pointRenderingModeCheckConnection =
        pointRenderingModeCheck.sigToggled().connect(
            bind(&SceneWidgetImpl::onPointRenderingModeToggled, impl, _1));
    hbox->addWidget(&pointRenderingModeCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    hbox = new QHBoxLayout();
    normalVisualizationCheck.setText(_("Normal Visualization"));
    normalVisualizationCheck.sigToggled().connect(
        bind(&SceneWidgetImpl::onNormalVisualizationChanged, impl));
    hbox->addWidget(&normalVisualizationCheck);
    normalLengthSpin.setDecimals(3);
    normalLengthSpin.setRange(0.0, 1000.0);
    normalLengthSpin.setSingleStep(0.001);
    normalLengthSpin.setValue(0.01);
    normalLengthSpin.sigValueChanged().connect(
        bind(&SceneWidgetImpl::onNormalVisualizationChanged, impl));
    hbox->addWidget(&normalLengthSpin);
    hbox->addStretch();
    vbox->addLayout(hbox);
    
    vbox->addWidget(new HSeparator());

    hbox = new QHBoxLayout();
    coordinateAxesCheck.setText(_("Coordinate axes"));
    coordinateAxesCheck.setChecked(true);
    coordinateAxesCheck.sigToggled().connect(
        bind(&SceneWidgetImpl::activateSystemNode, impl, boost::ref(impl->coordinateAxes), _1));
    hbox->addWidget(&coordinateAxesCheck);
    
    fpsCheck.setText(_("Show FPS"));
    fpsCheck.setChecked(false);
    fpsCheck.sigToggled().connect(bind(&SceneWidgetImpl::showFPS, impl, _1));
    hbox->addWidget(&fpsCheck);

    fpsTestButton.setText(_("Test"));
    fpsTestButton.sigClicked().connect(bind(&SceneWidgetImpl::doFPSTest, impl));
    hbox->addWidget(&fpsTestButton);
    hbox->addStretch();
    vbox->addLayout(hbox);

    hbox = new QHBoxLayout();
    bufferForPickingCheck.setText(_("Use an OpenGL pixel buffer for picking"));
    bufferForPickingCheck.setChecked(true);
    bufferForPickingCheck.sigToggled().connect(bind(&SceneWidgetImpl::onBufferForPickingToggled, impl, _1));
    hbox->addWidget(&bufferForPickingCheck);
    hbox->addStretch();
    vbox->addLayout(hbox);

    topVBox->addLayout(vbox);

    topVBox->addWidget(new HSeparator());
    QPushButton* okButton = new QPushButton(_("&Ok"));
    okButton->setDefault(true);
    QDialogButtonBox* buttonBox = new QDialogButtonBox(this);
    buttonBox->addButton(okButton, QDialogButtonBox::AcceptRole);
    connect(buttonBox,SIGNAL(accepted()), this, SLOT(accept()));
    topVBox->addWidget(buttonBox);
    
    setLayout(topVBox);
}


void SetupDialog::storeState(Archive& archive)
{
    archive.write("defaultHeadLight", headLightCheck.isChecked());
    archive.write("defaultHeadLightIntensity", headLightIntensitySpin.value());
    archive.write("headLightLightingFromBack", headLightFromBackCheck.isChecked());
    archive.write("worldLight", worldLightCheck.isChecked());
    archive.write("worldLightIntensity", worldLightIntensitySpin.value());
    archive.write("worldLightAmbient", worldLightAmbientSpin.value());
    archive.write("additionalLights", additionalLightsCheck.isChecked());
    archive.write("floorGrid", floorGridCheck.isChecked());
    archive.write("floorGridSpan", floorGridSpanSpin.value());
    archive.write("floorGridInterval", floorGridIntervalSpin.value());
    archive.write("texture", textureCheck.isChecked());
    archive.write("lineWidth", lineWidthSpin.value());
    archive.write("pointSize", pointSizeSpin.value());
    archive.write("normalVisualization", normalVisualizationCheck.isChecked());
    archive.write("normalLength", normalLengthSpin.value());
    archive.write("coordinateAxes", coordinateAxesCheck.isChecked());
    archive.write("showFPS", fpsCheck.isChecked());
    archive.write("useBufferForPicking", bufferForPickingCheck.isChecked());
}


void SetupDialog::restoreState(const Archive& archive)
{
    headLightCheck.setChecked(archive.get("defaultHeadLight", headLightCheck.isChecked()));
    headLightIntensitySpin.setValue(archive.get("defaultHeadLightIntensity", headLightIntensitySpin.value()));
    headLightFromBackCheck.setChecked(archive.get("headLightLightingFromBack", headLightFromBackCheck.isChecked()));
    worldLightCheck.setChecked(archive.get("worldLight", worldLightCheck.isChecked()));
    worldLightIntensitySpin.setValue(archive.get("worldLightIntensity", worldLightIntensitySpin.value()));
    worldLightAmbientSpin.setValue(archive.get("worldLightAmbient", worldLightAmbientSpin.value()));
    additionalLightsCheck.setChecked(archive.get("additionalLights", additionalLightsCheck.isChecked()));
    floorGridCheck.setChecked(archive.get("floorGrid", floorGridCheck.isChecked()));
    floorGridSpanSpin.setValue(archive.get("floorGridSpan", floorGridSpanSpin.value()));
    floorGridIntervalSpin.setValue(archive.get("floorGridInterval", floorGridIntervalSpin.value()));
    textureCheck.setChecked(archive.get("texture", textureCheck.isChecked()));
    lineWidthSpin.setValue(archive.get("lineWidth", lineWidthSpin.value()));
    pointSizeSpin.setValue(archive.get("pointSize", pointSizeSpin.value()));
    normalVisualizationCheck.setChecked(archive.get("normalVisualization", normalVisualizationCheck.isChecked()));
    normalLengthSpin.setValue(archive.get("normalLength", normalLengthSpin.value()));
    coordinateAxesCheck.setChecked(archive.get("coordinateAxes", coordinateAxesCheck.isChecked()));
    fpsCheck.setChecked(archive.get("showFPS", fpsCheck.isChecked()));
    bufferForPickingCheck.setChecked(archive.get("useBufferForPicking", bufferForPickingCheck.isChecked()));
}
