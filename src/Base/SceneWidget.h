/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_H
#define CNOID_BASE_SCENE_WIDGET_H

#include <cnoid/SceneGraph>
#include <cnoid/Widget>
#include <QBoxLayout>
#include "exportdecl.h"

namespace cnoid {

class ExtensionManager;
class SceneRenderer;
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

    SceneWidget(QWidget* parent);
    ~SceneWidget();

    static void forEachInstance(SgNode* node, std::function<void(SceneWidget* sceneWidget, const SgNodePath& path)> function);

    void setModeSyncEnabled(bool on);

    SceneWidgetRoot* sceneRoot();
    SgGroup* scene();
    SgGroup* systemNodeGroup();

    SceneRenderer* renderer();

    void renderScene(bool doImmediately = false);

    [[deprecated("Use renderScene(true).")]]
    void draw() { renderScene(true); }

    SignalProxy<void()> sigStateChanged() const;

    void setEditMode(bool on);
    bool isEditMode() const;

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

    void setHeadLightIntensity(double value);
    void setWorldLightIntensity(double value);
    void setWorldLightAmbient(double value);
    void setFloorGridSpan(double value);
    void setFloorGridInterval(double value);
    void setLineWidth(double value);
    void setPointSize(double value);
    void setNormalLength(double value);

    void setHeadLightEnabled(bool on);
    void setHeadLightLightingFromBack(bool on);
    void setWorldLight(bool on);
    void setAdditionalLights(bool on);
    void setFloorGrid(bool on);
    void setNormalVisualization(bool on);
    void setCoordinateAxes(bool on);
    void setShowFPS(bool on);
       
    void setBackgroundColor(const Vector3& color);
    Vector3 backgroundColor();
    void setColor(const Vector4& color);

    void setCameraPosition(const Vector3& eye, const Vector3& direction, const Vector3& up);
    void setFieldOfView(double value);
    void setHeight(double value);
    void setNear(double value);
    void setFar(double value);
 
    bool setSceneFocus(const SgNodePath& path);

    /**
       @return cursor id which is passed to releaseCursor()
    */
    //int setCursor(const QCursor cursor);
    //void releaseCursor(int cursorId);

    void setCursor(const QCursor cursor);

    Menu* contextMenu();
    void showContextMenuAtPointerPosition();
    SignalProxy<void(SceneWidgetEvent* event, MenuManager* menuManager)> sigContextMenuRequest();

    void showConfigDialog();
    QVBoxLayout* configDialogVBox();

    bool saveImage(const std::string& filename);
    QImage getImage();
    void setScreenSize(int width, int height);

    void updateIndicator(const std::string& text);
    QWidget* indicator();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    SignalProxy<void(bool isFocused)> sigWidgetFocusChanged();
    SignalProxy<void()> sigAboutToBeDestroyed();

    class Impl;

private:
    Impl* impl;
};


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
