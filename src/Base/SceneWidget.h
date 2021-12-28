/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_H
#define CNOID_BASE_SCENE_WIDGET_H

#include <cnoid/SceneGraph>
#include <cnoid/Widget>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class SceneRenderer;
class GLSceneRenderer;
class Archive;
class MenuManager;
class SceneWidgetEvent;
class SceneWidgetEventHandler;
class SceneWidgetRoot;
class Menu;
class InteractiveCameraTransform;

class CNOID_EXPORT SceneWidget : public Widget
{
public:
    static void initializeClass(ExtensionManager* ext);
    static SignalProxy<void(SceneWidget*)> sigSceneWidgetCreated();
    static bool isVerticalSyncMode();
    static void setVerticalSyncMode(bool on);
    static bool isLowMemoryConsumptionMode();
    static void setLowMemoryConsumptionMode(bool on);

    SceneWidget(QWidget* parent);
    ~SceneWidget();

    static void forEachInstance(SgNode* node, std::function<void(SceneWidget* sceneWidget, const SgNodePath& path)> function);

    void setModeSyncEnabled(bool on);

    SceneWidgetRoot* sceneRoot();
    SgGroup* scene();
    SgGroup* systemNodeGroup();

    SceneRenderer* renderer();
    template<class Renderer> Renderer* renderer();

    void renderScene(bool doImmediately = false);

    [[deprecated("Use renderScene(true).")]]
    void draw() { renderScene(true); }

    SignalProxy<void()> sigStateChanged() const;

    void setEditMode(bool on);
    bool isEditMode() const;

    void blockEditMode(Referenced* requester);
    void unblockEditMode(Referenced* requester);

    // Issue a unique ID from 2.
    // ID 0 is used as the non-custom mode ID.
    // ID 1 is used as the default (common) customo mode ID that can be used for any customo mode.
    static int issueUniqueCustomModeId();

    void activateCustomMode(SceneWidgetEventHandler* modeHandler, int modeId = 1);
    SceneWidgetEventHandler* activeCustomModeHandler();
    int activeCustomMode() const;
    // If modeHandler is nullptr, any current custom mode is deactivated.
    void deactivateCustomMode(SceneWidgetEventHandler* modeHandler = nullptr);

    SceneWidgetEvent* latestEvent();
    Vector3 lastClickedPoint() const;

    enum ViewpointOperationMode {
        ThirdPersonMode,
        FirstPersonMode
    };
    void setViewpointOperationMode(ViewpointOperationMode mode);
    ViewpointOperationMode viewpointOperationMode() const;

    SgPosTransform* builtinCameraTransform(void);
    SgPerspectiveCamera* builtinPerspectiveCamera() const;
    SgOrthographicCamera* builtinOrthographicCamera() const;
    bool isBuiltinCameraCurrent() const;
    bool isBuiltinCamera(SgCamera* camera) const;
    InteractiveCameraTransform* findOwnerInteractiveCameraTransform(int cameraIndex);
    InteractiveCameraTransform* activeInteractiveCameraTransform();

    void startBuiltinCameraViewChange(const Vector3& center);
    void rotateBuiltinCameraView(double dPitch, double dYaw);
    void translateBuiltinCameraView(const Vector3& dp_local);

    bool unproject(double x, double y, double z, Vector3& out_projected) const;
        
    void viewAll();

    // This is same as SgPolygonDrawStyle::PolygonElement
    enum PolygonElement {
        PolygonFace = 1,
        PolygonEdge = 2,
        PolygonVertex = 4
    };
    void setVisiblePolygonElements(int elementFlags);
    int visiblePolygonElements() const;

    void setHighlightingEnabled(bool on);
    bool isHighlightingEnabled() const;

    void setCollisionLineVisibility(bool on);
    bool collisionLineVisibility() const;

    void setHeadLightIntensity(double intensity);
    void setHeadLightEnabled(bool on);
    void setHeadLightLightingFromBack(bool on);
    void setAdditionalLights(bool on);

    void setLineWidth(double width);
    void setPointSize(double size);

    enum GridPlane { XY_Grid = 0, XZ_Grid = 1, YZ_Grid = 2 };
    void setGridEnabled(GridPlane plane, bool on);
    bool isGridEnabled(GridPlane plane) const;
    void setGridGeometry(GridPlane plane, double span, double interval);
    void setGridColor(GridPlane plane, const Vector3f& color);
    void updateGrids();

    void setCoordinateAxes(bool on);
    void setShowFPS(bool on);

    // Make the following three functions deprecated?
    void setBackgroundColor(const Vector3& color);
    Vector3 backgroundColor();
    void setColor(const Vector4& color);

    void setCameraPosition(const Vector3& eye, const Vector3& direction, const Vector3& up);
    void setFieldOfView(double value);
    void setClipDistances(double nearDistance, double farDistance);
    void setInteractiveCameraRollRestricted(bool on);
    void setVerticalAxis(int axis);
    void setLightweightViewChangeEnabled(bool on);
    void setHeight(double value);

    bool setSceneFocus(const SgNodePath& path);

    /**
       @return cursor id which is passed to releaseCursor()
    */
    //int setCursor(const QCursor cursor);
    //void releaseCursor(int cursorId);

    void setCursor(const QCursor cursor);

    Menu* contextMenu();
    void showContextMenuAtPointerPosition();
    SignalProxy<void(SceneWidgetEvent* event, MenuManager* menu)> sigContextMenuRequest();

    typedef std::function<bool(SgNode* node, SceneWidgetEventHandler* orgHandler, SceneWidgetEvent* event)>
        NodeEventHandler;
    /**
       This is used to override the event handler for a particular node type.
       @return Return true if the overridden handler is processed. Otherwise, return false
       and the original handler will be processed.
    */
    void overrideNodeEventHandler(NodeEventHandler handler);

    bool saveImage(const std::string& filename);
    QImage getImage();
    void setScreenSize(int width, int height);

    void updateIndicator(const std::string& text);
    QWidget* indicator();

    void doFpsTest(int iteration);
    void cancelFpsTest();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    SignalProxy<void(bool isFocused)> sigWidgetFocusChanged();
    SignalProxy<void()> sigAboutToBeDestroyed();

    [[deprecated("This function does nothing.")]]
    void setWorldLightEnabled(bool on);
    [[deprecated]]
    bool isWorldLightEnabled() const;
    [[deprecated("This function does nothing.")]]
    void setWorldLightIntensity(double intensity);
    [[deprecated("Use renderer()->headLight()->setAmbientIntensity()")]]
    void setWorldLightAmbient(double intensity);
    [[deprecated("Use GLSceneRenderer::setNormalVisualizationEnabled.")]]
    void setNormalVisualization(bool on);
    [[deprecated("Use GLSceneRenderer::setNormalVisualizationLength.")]]
    void setNormalLength(double value);
    [[deprecated("Use setGridEnabled")]]
    void setFloorGridEnabled(bool on);
    [[deprecated("Use isGridEnabled")]]
    bool isFloorGridEnabled() const;
    [[deprecated("Use setGridGeometry")]]
    void setFloorGridSpan(double span);
    [[deprecated("Use setGridGeometry")]]
    void setFloorGridInterval(double intervale);
 
    class Impl;

private:
    Impl* impl;
};

template<> CNOID_EXPORT GLSceneRenderer* SceneWidget::renderer<GLSceneRenderer>();


class CNOID_EXPORT SceneWidgetRoot : public SgGroup
{
public:
    SceneWidget* sceneWidget() { return sceneWidget_; }
private:
    SceneWidgetRoot(SceneWidget* sceneWidget);
    SceneWidget* sceneWidget_;
    SgUnpickableGroupPtr systemGroup;
    friend class SceneWidget;
};
typedef ref_ptr<SceneWidgetRoot> SceneWidgetRootPtr;

}

#endif
