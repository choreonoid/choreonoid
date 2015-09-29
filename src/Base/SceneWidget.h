/**
   @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BASE_SCENE_WIDGET_H
#define CNOID_BASE_SCENE_WIDGET_H

#include <QWidget>
#include <cnoid/SceneGraph>
#include <QWidget>
#include <QBoxLayout>
#include "exportdecl.h"

namespace cnoid {

class SceneWidgetImpl;
class SceneRenderer;
class Archive;
class MenuManager;
class SceneWidgetEvent;
class SceneWidgetEditable;
class SceneWidgetRoot;
class Menu;
class InteractiveCameraTransform;

class CNOID_EXPORT SceneWidget : public QWidget
{
public:
    static SignalProxy<void(SceneWidget*)> sigSceneWidgetCreated();

    SceneWidget();
    ~SceneWidget();

    static void forEachInstance(SgNode* node, boost::function<void(SceneWidget* sceneWidget, const SgNodePath& path)> function);

    SceneWidgetRoot* sceneRoot();
    SgGroup* scene();

    SceneRenderer& renderer();

    SignalProxy<void()> sigStateChanged() const;

    void setEditMode(bool on);
    bool isEditMode() const;

    const SceneWidgetEvent& latestEvent() const;

    enum ViewpointControlMode { THIRD_PERSON_MODE, FIRST_PERSON_MODE  };
    void setViewpointControlMode(ViewpointControlMode mode);
    ViewpointControlMode viewpointControlMode() const;

    SgPosTransform* builtinCameraTransform(void);
    SgPerspectiveCamera* builtinPerspectiveCamera() const;
    SgOrthographicCamera* builtinOrthographicCamera() const;
    bool isBuiltinCameraCurrent() const;
    InteractiveCameraTransform* findOwnerInteractiveCameraTransform(int cameraIndex);

    void startBuiltinCameraViewChange(const Vector3& center);
    void rotateBuiltinCameraView(double dPitch, double dYaw);
    void translateBuiltinCameraView(const Vector3& dp_local);

    bool unproject(double x, double y, double z, Vector3& out_projected) const;
        
    void viewAll();

    enum PolygonMode { FILL_MODE, LINE_MODE, POINT_MODE };
    void setPolygonMode(PolygonMode mode);
    PolygonMode polygonMode() const;

    void setCollisionLinesVisible(bool on);
    bool collisionLinesVisible() const;

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
    void setNewDisplayListDoubleRenderingEnabled(bool on);
    void setUseBufferForPicking(bool on);
       
    void setBackgroundColor(const Vector3& color);
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
    void showContextMenu();
    SignalProxy<void(const SceneWidgetEvent& event, MenuManager& menuManager)> sigContextMenuRequest();

    void installEventFilter(SceneWidgetEditable* filter);
    SceneWidgetEditable* activeEventFilter();
    void removeEventFilter(SceneWidgetEditable* filter);

    void showSetupDialog();
    QVBoxLayout* setupDialogVBox();

    bool saveImage(const std::string& filename);
    QImage getImage();
    void setScreenSize(int width, int height);

    void updateIndicator(const std::string& text);
    QWidget* indicator();

    bool storeState(Archive& archive);
    bool restoreState(const Archive& archive);

    SignalProxy<void(bool isFocused)> sigWidgetFocusChanged();
    SignalProxy<void()> sigAboutToBeDestroyed();

private:
    SceneWidgetImpl* impl;
};


class CNOID_EXPORT SceneWidgetRoot : public SgGroup
{
public:
    SceneWidget* sceneWidget() { return sceneWidget_; }
private:
    SceneWidgetRoot(SceneWidget* sceneWidget);
    SceneWidget* sceneWidget_;
    SgGroupPtr systemGroup;
    friend class SceneWidgetImpl;
};
typedef ref_ptr<SceneWidgetRoot> SceneWidgetRootPtr;

}

#endif
